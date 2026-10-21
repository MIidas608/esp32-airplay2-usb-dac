#include "uac_output.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "usb/uac_host.h"

static const char *TAG = "uac_output";

#define USB_HOST_TASK_PRIORITY  5
#define UAC_TASK_PRIORITY       5
#define USER_TASK_PRIORITY      2

// AirPlay audio is 44.1 kHz. CX21988 supports it natively (alt[2], 24-bit,
// endpoint MPS 576 within the 600-byte periodic FIFO), so the stream is
// passed through without resampling.
#define TARGET_SAMPLE_FREQ      44100

// EQ: 5 peaking bands (Hz) — low / low-mid / mid / high-mid / high
static const float EQ_BAND_FREQS[UAC_EQ_BANDS] = { 80.0f, 250.0f, 1000.0f, 4000.0f, 12000.0f };
static const float EQ_BAND_Q[UAC_EQ_BANDS]     = { 0.9f, 0.9f, 0.9f, 0.9f, 0.9f };

// Biquad (Transposed Direct Form II) per channel per band
typedef struct {
    float b0, b1, b2, a1, a2;   // coefficients (a0 normalized to 1)
    float z1, z2;               // state
} biquad_t;

static struct {
    bool enabled;
    biquad_t bands[2][UAC_EQ_BANDS];  // [channel][band]
} s_eq = { .enabled = false };

/* User EQ gains (last configured) — loudness compensation is layered on
 * top of these for bands 0 (80 Hz) and 4 (12 kHz). */
static int8_t  s_user_gains[UAC_EQ_BANDS] = { 0, 0, 0, 0, 0 };
static bool    s_user_eq_enabled = false;
static bool    s_loudness = true;      /* default on */
static float   s_volume_db = -60.0f;   /* last known volume (for loudness) */

static void biquad_peaking(biquad_t *bq, float freq, float q, float gain_db, float fs);

// Loudness compensation: bass/treble boost as a function of volume.
// -20 dB and above: flat.  Between -20 and -60 dB the boost ramps
// linearly to +10 dB (80 Hz) and +5 dB (12 kHz), then clamps.
static void loudness_boosts(float db, float *bass_db, float *treb_db)
{
    if (db >= -20.0f) {
        *bass_db = 0.0f;
        *treb_db = 0.0f;
        return;
    }
    float depth = (-20.0f - db) / 40.0f;
    if (depth > 1.0f) depth = 1.0f;
    *bass_db = 10.0f * depth;
    *treb_db = 5.0f * depth;
}

// Recompute all biquad coefficients from the user EQ gains plus the
// volume-dependent loudness boost (bands 0 and 4).
static void eq_recompute(void)
{
    bool any = s_user_eq_enabled || s_loudness;
    if (s_eq.enabled && !any) {
        s_eq.enabled = false;
        ESP_LOGI(TAG, "EQ disabled");
        return;
    }
    if (any && !s_eq.enabled) {
        s_eq.enabled = true;
        ESP_LOGI(TAG, "EQ chain enabled");
    }
    if (!s_eq.enabled) {
        return;
    }
    float bass_db, treb_db;
    loudness_boosts(s_volume_db, &bass_db, &treb_db);
    float eff[UAC_EQ_BANDS];
    for (int b = 0; b < UAC_EQ_BANDS; b++) {
        eff[b] = (float)s_user_gains[b];
    }
    eff[0] += bass_db;
    eff[4] += treb_db;
    for (int ch = 0; ch < 2; ch++) {
        for (int b = 0; b < UAC_EQ_BANDS; b++) {
            biquad_peaking(&s_eq.bands[ch][b], EQ_BAND_FREQS[b], EQ_BAND_Q[b],
                           eff[b], TARGET_SAMPLE_FREQ);
        }
    }
    ESP_LOGD(TAG, "EQ recomputed: gains=[%.1f,%.1f,%.1f,%.1f,%.1f] dB (vol=%.1f, loudness=%d)",
             eff[0], eff[1], eff[2], eff[3], eff[4], s_volume_db, s_loudness);
}

void uac_output_eq_configure(bool enabled, const int8_t *gains_db)
{
    s_user_eq_enabled = enabled && gains_db;
    if (gains_db) {
        for (int b = 0; b < UAC_EQ_BANDS; b++) {
            s_user_gains[b] = gains_db[b];
        }
    } else {
        memset(s_user_gains, 0, sizeof(s_user_gains));
    }
    eq_recompute();
}

void uac_output_set_loudness(bool enabled)
{
    if (s_loudness == enabled) {
        return;
    }
    s_loudness = enabled;
    ESP_LOGI(TAG, "Loudness %s", enabled ? "enabled" : "disabled");
    eq_recompute();
}

// Generate peaking EQ coefficients (RBJ audio EQ cookbook)
static void biquad_peaking(biquad_t *bq, float freq, float q, float gain_db, float fs)
{
    float A = powf(10.0f, gain_db / 40.0f);
    float w0 = 2.0f * (float)M_PI * freq / fs;
    float alpha = sinf(w0) / (2.0f * q);

    float cosw0 = cosf(w0);
    float a0 = 1.0f + alpha / A;
    bq->b0 = (1.0f + alpha * A) / a0;
    bq->b1 = (-2.0f * cosw0) / a0;
    bq->b2 = (1.0f - alpha * A) / a0;
    bq->a1 = (-2.0f * cosw0) / a0;
    bq->a2 = (1.0f - alpha / A) / a0;
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

// In-place process one sample of a channel through the 5-band chain
static inline float eq_process_sample(float x, int ch)
{
    for (int b = 0; b < UAC_EQ_BANDS; b++) {
        biquad_t *q = &s_eq.bands[ch][b];
        float y = q->b0 * x + q->z1;
        q->z1 = q->b1 * x - q->a1 * y + q->z2;
        q->z2 = q->b2 * x - q->a2 * y;
        x = y;
    }
    return x;
}

static QueueHandle_t s_event_queue = NULL;
static uac_host_device_handle_t s_spk_dev_handle = NULL;
static volatile bool s_ready = false;

typedef enum {
    UAC_DRIVER_EVENT,
    UAC_DEVICE_EVENT,
} event_group_t;

typedef struct {
    event_group_t event_group;
    union {
        struct {
            uint8_t addr;
            uint8_t iface_num;
            uac_host_driver_event_t event;
            void *arg;
        } driver_evt;
        struct {
            uac_host_device_handle_t handle;
            uac_host_device_event_t event;
            void *arg;
        } device_evt;
    };
} s_event_queue_t;

static void uac_device_callback(uac_host_device_handle_t uac_device_handle, const uac_host_device_event_t event, void *arg)
{
    s_event_queue_t evt = {
        .event_group = UAC_DEVICE_EVENT,
        .device_evt.handle = uac_device_handle,
        .device_evt.event = event,
        .device_evt.arg = arg
    };
    xQueueSend(s_event_queue, &evt, 0);
}

static void uac_host_lib_callback(uint8_t addr, uint8_t iface_num, const uac_host_driver_event_t event, void *arg)
{
    s_event_queue_t evt = {
        .event_group = UAC_DRIVER_EVENT,
        .driver_evt.addr = addr,
        .driver_evt.iface_num = iface_num,
        .driver_evt.event = event,
        .driver_evt.arg = arg
    };
    xQueueSend(s_event_queue, &evt, 0);
}

static bool find_dev_alt_params_for_freq(uac_host_device_handle_t handle, uint32_t freq,
                                         uac_host_dev_alt_param_t *out)
{
    uac_host_dev_info_t info;
    if (uac_host_get_device_info(handle, &info) != ESP_OK) {
        return false;
    }
    // Prefer higher bit resolution (24bit, EP MPS=576) so the endpoint fits the
    // ESP32-S3 periodic OUT FIFO (~600 bytes after biasing).
    bool found = false;
    uac_host_dev_alt_param_t best;
    for (uint8_t alt = 1; alt <= info.iface_alt_num; alt++) {
        uac_host_dev_alt_param_t p;
        if (uac_host_get_device_alt_param(handle, alt, &p) != ESP_OK) {
            continue;
        }
        bool match = false;
        if (p.sample_freq_type > 0) {
            for (int i = 0; i < p.sample_freq_type; i++) {
                if (p.sample_freq[i] == freq) {
                    match = true;
                    break;
                }
            }
        } else {
            match = (freq >= p.sample_freq_lower && freq <= p.sample_freq_upper);
        }
        if (match) {
            if (!found || p.bit_resolution > best.bit_resolution) {
                best = p;
                found = true;
            }
        }
    }
    if (found) {
        *out = best;
    }
    return found;
}

static void usb_lib_task(void *arg)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host installed");
    xTaskNotifyGive((TaskHandle_t)arg);

    while (true) {
        uint32_t event_flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
            break;
        }
    }
    ESP_LOGI(TAG, "USB Host shutdown");
    vTaskDelay(10);
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(NULL);
}

static void uac_lib_task(void *arg)
{
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uac_host_driver_config_t uac_config = {
        .create_background_task = true,
        .task_priority = 18,
        .stack_size = 4096,
        .core_id = 0,
        .callback = uac_host_lib_callback,
        .callback_arg = NULL
    };
    ESP_ERROR_CHECK(uac_host_install(&uac_config));
    ESP_LOGI(TAG, "UAC driver installed, waiting for DAC...");

    s_event_queue_t evt = {0};
    while (1) {
        if (xQueueReceive(s_event_queue, &evt, portMAX_DELAY)) {
            if (UAC_DRIVER_EVENT == evt.event_group) {
                if (evt.driver_evt.event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
                    uac_host_device_handle_t dev = NULL;
                    const uac_host_device_config_t dev_config = {
                        .addr = evt.driver_evt.addr,
                        .iface_num = evt.driver_evt.iface_num,
                        .buffer_size = 48000,
                        .buffer_threshold = 8000,
                        .callback = uac_device_callback,
                        .callback_arg = NULL,
                    };
                    ESP_ERROR_CHECK(uac_host_device_open(&dev_config, &dev));
                    uac_host_printf_device_param(dev);

                    uac_host_dev_alt_param_t alt_params;
                    if (!find_dev_alt_params_for_freq(dev, TARGET_SAMPLE_FREQ, &alt_params)) {
                        ESP_LOGE(TAG, "%u Hz not supported by DAC!", TARGET_SAMPLE_FREQ);
                        uac_host_device_close(dev);
                        break;
                    }
                    uac_host_stream_config_t stm = {
                        .channels = alt_params.channels,
                        .bit_resolution = alt_params.bit_resolution,
                        .sample_freq = TARGET_SAMPLE_FREQ,
                    };
                    ESP_LOGI(TAG, "Start UAC speaker: %lu Hz, %u bits, %s",
                             (unsigned long)stm.sample_freq, stm.bit_resolution,
                             stm.channels == 1 ? "Mono" : "Stereo");
                    if (uac_host_device_start(dev, &stm) != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to start UAC stream");
                        uac_host_device_close(dev);
                        break;
                    }
                    s_spk_dev_handle = dev;
                    uac_host_device_set_volume(dev, 80);
                    uac_host_device_set_mute(dev, false);
                    s_ready = true;
                    ESP_LOGI(TAG, "UAC output ready");
                }
            } else if (UAC_DEVICE_EVENT == evt.event_group) {
                if (evt.device_evt.event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
                    ESP_LOGW(TAG, "DAC disconnected");
                    if (evt.device_evt.handle == s_spk_dev_handle) {
                        s_spk_dev_handle = NULL;
                        s_ready = false;
                    }
                    uac_host_device_close(evt.device_evt.handle);
                }
            }
        }
    }
}

void uac_output_init(void)
{
    s_event_queue = xQueueCreate(10, sizeof(s_event_queue_t));
    assert(s_event_queue != NULL);

    static TaskHandle_t uac_task_handle = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(uac_lib_task, "uac_events", 4096, NULL,
                                             USER_TASK_PRIORITY, &uac_task_handle, 0);
    assert(ret == pdTRUE);
    ret = xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096, (void *)uac_task_handle,
                                  USB_HOST_TASK_PRIORITY, NULL, 0);
    assert(ret == pdTRUE);
}

void uac_output_write(const uint8_t *data, size_t len)
{
    if (!s_ready || !s_spk_dev_handle) {
        /* DAC not enumerated yet: yield briefly so the caller's loop
         * doesn't spin one core and starve the idle task (watchdog). */
        vTaskDelay(pdMS_TO_TICKS(2));
        return;
    }

    /* Rate gate: audio_receiver_read() is non-blocking, so the pacing has
     * to happen here. The delay is quantised to whole milliseconds and
     * re-based on the wall clock each frame, which makes playback run
     * slightly slower than real time (~4%). The resulting constant lag
     * keeps frames early relative to their scheduled play time, so WiFi
     * jitter is absorbed by the queue instead of being dropped as late
     * frames. Track changes flush the buffer, so the lag does not
     * accumulate. */
    size_t frames = len / 4;  /* stereo int16: 4 bytes per frame */
    static int64_t s_next_slot_us = 0;
    int64_t now_us = esp_timer_get_time();
    int64_t frame_us = (int64_t)frames * 1000000LL / TARGET_SAMPLE_FREQ;
    if (now_us < s_next_slot_us) {
        int64_t wait_us = s_next_slot_us - now_us;
        vTaskDelay(pdMS_TO_TICKS((wait_us + 999) / 1000));
    }
    s_next_slot_us = esp_timer_get_time() + frame_us;

    // Convert 16-bit stereo interleaved PCM to 24-bit (3 bytes per sample, little-endian)
    size_t samples = len / 2;
    size_t out_len = samples * 3;
    static uint8_t *conv_buf = NULL;
    static size_t conv_size = 0;
    if (conv_size < out_len) {
        uint8_t *nb = realloc(conv_buf, out_len);
        if (!nb) {
            ESP_LOGE(TAG, "conv buf alloc failed");
            return;
        }
        conv_buf = nb;
        conv_size = out_len;
    }
    const int16_t *in = (const int16_t *)data;
    const bool eq_on = s_eq.enabled;
    for (size_t i = 0; i < samples; i++) {
        int32_t s;
        if (eq_on) {
            int ch = (int)(i & 1u);   // interleaved stereo: even=L, odd=R
            float x = eq_process_sample((float)in[i], ch);
            if (x > 32767.0f) x = 32767.0f;
            if (x < -32768.0f) x = -32768.0f;
            s = ((int32_t)(int16_t)x) << 8;
        } else {
            s = ((int32_t)in[i]) << 8;
        }
        conv_buf[i * 3]     = (uint8_t)(s & 0xFF);
        conv_buf[i * 3 + 1] = (uint8_t)((s >> 8) & 0xFF);
        conv_buf[i * 3 + 2] = (uint8_t)((s >> 16) & 0xFF);
    }

    // Diagnostics: measure write blocking and failure rate
    static uint32_t stat_calls = 0;
    static uint32_t stat_fail = 0;
    static int64_t stat_block_us = 0;
    static int64_t stat_max_us = 0;
    static int64_t stat_last_ms = 0;

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = uac_host_device_write(s_spk_dev_handle, conv_buf, out_len, 1000);
    int64_t dt = esp_timer_get_time() - t0;
    stat_calls++;
    if (err != ESP_OK) {
        stat_fail++;
    }
    stat_block_us += dt;
    if (dt > stat_max_us) {
        stat_max_us = dt;
    }
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms - stat_last_ms > 5000) {
        ESP_LOGD(TAG, "UAC diag: calls=%lu fail=%lu avg_wr_us=%lld max_wr_us=%lld len=%u",
                 (unsigned long)stat_calls, (unsigned long)stat_fail,
                 (long long)(stat_calls ? stat_block_us / stat_calls : 0),
                 (long long)stat_max_us, (unsigned)out_len);
        stat_calls = 0;
        stat_fail = 0;
        stat_block_us = 0;
        stat_max_us = 0;
        stat_last_ms = now_ms;
    }
}

void uac_output_deinit(void)
{
    s_ready = false;
    if (s_spk_dev_handle) {
        uac_host_device_stop(s_spk_dev_handle);
        uac_host_device_close(s_spk_dev_handle);
        s_spk_dev_handle = NULL;
    }
}

void uac_output_set_volume_db(float volume_db)
{
    // Clamp to a sane range (most UAC DACs support at least -60..0 dB)
    if (volume_db < -60.0f) volume_db = -60.0f;
    if (volume_db > 0.0f)   volume_db = 0.0f;

    // Track volume even before the DAC is up: loudness compensation and
    // volume restore both depend on it.
    if (volume_db != s_volume_db) {
        s_volume_db = volume_db;
        eq_recompute();   /* loudness boost follows the volume */
    }

    if (!s_spk_dev_handle) {
        return;
    }
    // UAC volume is in 1/256 dB units
    int16_t db256 = (int16_t)(volume_db * 256.0f);
    esp_err_t err = uac_host_device_set_volume_db(s_spk_dev_handle, db256);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_volume_db(%.1f dB): %s", volume_db, esp_err_to_name(err));
    }
}

void uac_output_set_mute(bool mute)
{
    if (!s_spk_dev_handle) {
        return;
    }
    esp_err_t err = uac_host_device_set_mute(s_spk_dev_handle, mute);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_mute(%d): %s", mute, esp_err_to_name(err));
    }
}

bool uac_output_is_ready(void)
{
    return s_ready;
}
