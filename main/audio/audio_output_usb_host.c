/**
 * USB Host UAC output — ESP32 drives an external USB DAC (CX21988 dongle)
 *
 * The ESP32 acts as a USB Host and streams decoded AirPlay PCM to a UAC 1.0
 * speaker device (e.g. the CX21988 headphone dongle used in this project).
 * Streaming is handled by the uac_output component, which wraps
 * espressif/usb_host_uac.
 */

#include "audio_output.h"

#include "audio_receiver.h"
#include "audio_resample.h"
#include "uac_output.h"
#include "rtsp_server.h"
#include "settings.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TAG           "audio_usb_host"
#define OUTPUT_RATE   CONFIG_OUTPUT_SAMPLE_RATE_HZ
#define FRAME_SAMPLES 352

/* Max output frames after resampling one input frame */
#define MAX_RESAMPLE_FRAMES \
  ((size_t)((FRAME_SAMPLES + 2) * ((double)OUTPUT_RATE / 44100) + 16))

#if CONFIG_FREERTOS_UNICORE
#define PLAYBACK_CORE 0
#else
#define PLAYBACK_CORE 1
#endif

static volatile bool flush_requested = false;
static volatile int source_rate = 44100;
static volatile bool resample_reinit_needed = false;
static volatile int32_t last_vol_q15 = -1;

/* Volume restore: apply the saved volume once the DAC is up. */
static volatile bool restore_volume_pending = false;
static volatile float restore_volume_db = -60.0f;

/* Stalled-playback watchdog: after a long pause (>~2 min) the PTP clock
 * can be stale, leaving the resume anchor with a huge lead so playout
 * never starts — the engine reports playing but read() yields nothing.
 * If that persists for more than 8 s, force a seek_flush to re-anchor. */
static int64_t last_data_us = 0;

/* Volume persistence debounce: sliding the volume bar produces a burst of
 * q15 changes, and committing NVS on every step would wear the flash. */
static int64_t last_vol_change_us = 0;

/* Playback task: pulls decoded PCM from the shared receiver, resamples to
 * the output rate, maps AirPlay volume to the DAC hardware volume, and
 * hands the buffer to uac_output_write() for 16->24 bit conversion and
 * USB isochronous streaming. */
static void playback_task(void *arg) {
  int16_t *pcm = malloc((size_t)(FRAME_SAMPLES + 1) * 2 * sizeof(int16_t));
  int16_t *resample_buf = malloc(MAX_RESAMPLE_FRAMES * 2 * sizeof(int16_t));
  if (!pcm || !resample_buf) {
    ESP_LOGE(TAG, "Failed to allocate buffers");
    free(pcm);
    free(resample_buf);
    vTaskDelete(NULL);
    return;
  }

  while (true) {
    if (resample_reinit_needed) {
      resample_reinit_needed = false;
      audio_resample_init((uint32_t)source_rate, OUTPUT_RATE, 2);
    }
    if (flush_requested) {
      flush_requested = false;
      audio_resample_reset();
    }

    size_t samples = audio_receiver_read(pcm, FRAME_SAMPLES + 1);
    if (samples > 0) {
      last_data_us = esp_timer_get_time();
      int16_t *play_buf = pcm;
      size_t play_samples = samples;
      if (audio_resample_is_active()) {
        play_samples = audio_resample_process(pcm, samples, resample_buf,
                                              MAX_RESAMPLE_FRAMES);
        play_buf = resample_buf;
      }

      /* Hardware volume: poll AirPlay volume (q15) and mirror it on the
       * DAC's UAC volume feature.  Only issues a control transfer when the
       * value actually changes.  Skipping the software apply_volume keeps
       * the full 16-bit resolution at every volume level. */
      int32_t vol = airplay_get_volume_q15();

      /* Volume restore: once the DAC is enumerated and before the first
       * volume comes in, apply the saved volume from the last session. */
      if (restore_volume_pending && uac_output_is_ready()) {
        restore_volume_pending = false;
        uac_output_set_volume_db((float)restore_volume_db);
        ESP_LOGI(TAG, "Restored volume: %.1f dB", (double)restore_volume_db);
      }

      if (vol != last_vol_q15) {
        last_vol_q15 = vol;
        double db = 20.0 * log10((double)(vol > 0 ? vol : 1) / 32768.0);
        if (db < -60.0) db = -60.0;
        if (db > 0.0) db = 0.0;
        ESP_LOGD(TAG, "AirPlay volume q15=%ld -> %.1f dB (DAC hardware)", (long)vol, db);
        uac_output_set_volume_db((float)db);
        /* Cache only — persistence is debounced (see loop bottom). */
        settings_set_volume((float)db);
        last_vol_change_us = esp_timer_get_time();
      }

      uac_output_write((const uint8_t *)play_buf, play_samples * 4);

      /* No delay here: uac_output_write() rate-gates to the output sample
       * rate and blocks long enough for the idle task to feed the watchdog.
       * An additional delay here would make the frames go stale. */
    } else {
      /* Stalled-playback watchdog (see above): engine says playing but no
       * frames for 8 s → re-anchor. */
      if (audio_receiver_is_playing() && last_data_us > 0 &&
          (esp_timer_get_time() - last_data_us) > 8000000LL) {
        ESP_LOGW(TAG, "Playback stalled 8s while playing — forcing re-anchor");
        audio_receiver_seek_flush();
        last_data_us = esp_timer_get_time();
      }
      vTaskDelay(1);
    }

    /* Volume debounce: persist once ~3 s after the last change so a burst
     * of changes coalesces into a single NVS write. Runs on every loop
     * iteration, so it also fires while paused. */
    if (last_vol_change_us > 0 &&
        (esp_timer_get_time() - last_vol_change_us) > 3000000LL) {
      settings_persist_volume();
      last_vol_change_us = 0;
    }
  }
}

/* Public API */

esp_err_t audio_output_init(void) {
  ESP_LOGI(TAG, "Initialising USB Host UAC output (rate=%d)", OUTPUT_RATE);

  /* Volume memory: remember the last session's volume for restore once
   * the DAC is up (settings_set_volume only caches; we persist on change). */
  float saved = -60.0f;
  if (settings_get_volume(&saved) == ESP_OK) {
    restore_volume_db = saved;
    restore_volume_pending = true;
    ESP_LOGI(TAG, "Saved volume found: %.1f dB (will restore)", (double)saved);
  }

  /* Starts the USB Host + UAC driver and waits (async) for the DAC to
   * enumerate.  The UAC stream itself is configured to OUTPUT_RATE. */
  uac_output_init();

  audio_resample_init(44100, OUTPUT_RATE, 2);
  ESP_LOGI(TAG, "USB Host UAC output ready");
  return ESP_OK;
}

void audio_output_start(void) {
  /* Deep call chain: uac_output_write -> uac_host_device_write -> USB
   * driver.  16KB stack is required (learned the hard way in the
   * AirPlay 1 firmware — a 4KB task stack overflowed). */
  xTaskCreatePinnedToCore(playback_task, "uac_play", 16384, NULL,
                          AUDIO_PLAYBACK_TASK_PRIORITY, NULL, PLAYBACK_CORE);
}

void audio_output_flush(void) {
  flush_requested = true;
}

void audio_output_stop(void) {
  /* No-op: USB output owns the bus, nothing to yield. */
}

void audio_output_set_source_rate(int rate) {
  if (rate > 0 && rate != source_rate) {
    source_rate = rate;
    resample_reinit_needed = true;
  }
}

uint32_t audio_output_get_hardware_latency_us(void) {
  /* USB isochronous endpoint double buffering + UAC internal ring. */
  return 2000;
}

/* The uac_output component exposes no transfer-completion cursor or
 * underrun counter, and channel routing is fixed by the DAC, so the weak
 * defaults in audio_output_common.c cover the rest of the API. */
