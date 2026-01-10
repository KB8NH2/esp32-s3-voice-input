#ifndef STT_H
#define STT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send PCM16 mono 16 kHz audio to a REST Whisper server using
 *        multipart/form-data with a WAV wrapper.
 *
 * This function:
 *   - Builds a 44-byte WAV header
 *   - Wraps audio in multipart/form-data
 *   - Sends HTTP POST to /asr
 *   - Returns transcription text (caller must free)
 *
 * @param pcm_data      Pointer to PCM16 audio samples
 * @param pcm_samples   Number of samples (e.g., 16000 for 1 second)
 *
 * @return char*        JSON response string or NULL on error
 *                      Caller must free() the returned buffer.
 */
char *stt_send_wav_multipart(const int16_t *pcm_data, size_t pcm_samples);

#ifdef __cplusplus
}
#endif

#endif // STT_H