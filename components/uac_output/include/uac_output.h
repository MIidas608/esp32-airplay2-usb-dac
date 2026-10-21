#ifndef UAC_OUTPUT_H
#define UAC_OUTPUT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Number of EQ bands used by the USB output path. The web EQ exposes the
// same five bands (80 / 250 / 1k / 4k / 12 kHz).
#define UAC_EQ_BANDS 5
#define UAC_EQ_GAIN_MIN_DB (-12)
#define UAC_EQ_GAIN_MAX_DB 12

/**
 * @brief Initialize USB Host + UAC driver and wait for DAC.
 *        Must be called once before uac_output_write().
 */
void uac_output_init(void);

/**
 * @brief Write PCM audio data to the USB DAC.
 *
 * @param data 44.1kHz/16bit stereo interleaved PCM (same format as I2S path)
 * @param len Length in bytes
 */
void uac_output_write(const uint8_t *data, size_t len);

// Stop and release USB resources
void uac_output_deinit(void);

/**
 * @brief Set DAC hardware volume (UAC volume feature), dB scale.
 * @param volume_db dB value (e.g. 0.0f = max, -20.0f = 20dB attenuation).
 *                  Mapped to the DAC's actual volume range.
 */
void uac_output_set_volume_db(float volume_db);

/**
 * @brief Mute/unmute the DAC (UAC mute feature).
 */
void uac_output_set_mute(bool mute);

/**
 * @brief True when the UAC stream is active and ready for writes.
 */
bool uac_output_is_ready(void);

/**
 * @brief Configure the 5-band EQ. Gains in dB per band, -12..+12.
 *        Call before/after streaming; takes effect on next write.
 */
void uac_output_eq_configure(bool enabled, const int8_t *gains_db);

/**
 * @brief Enable/disable loudness compensation (auto bass/treble boost at
 *        low volume, per the equal-loudness contour).  When enabled, the
 *        effective gain of band 0 (80 Hz) and band 4 (12 kHz) is the user
 *        EQ gain plus a volume-dependent boost (0 dB at >= -20 dB, up to
 *        +10 dB / +5 dB at <= -60 dB).  Default: enabled.
 */
void uac_output_set_loudness(bool enabled);

#endif // UAC_OUTPUT_H
