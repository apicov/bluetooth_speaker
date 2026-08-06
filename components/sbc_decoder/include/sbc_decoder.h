/*
 * SBC decoding for the dancefloor.
 *
 * Wraps the OI decoder vendored from Bluedroid so callers deal in "here is an
 * SBC frame, give me PCM" rather than the codec's own conventions.
 *
 * Why this exists: the bridge forwards SBC frames untouched instead of decoding
 * them, so the inter-chip link and the WiFi stream carry ~330 kbps rather than
 * 1.4 Mbps. Each unit decodes at the point of playback. Quality is unaffected --
 * it is the same SBC, decoded once, just later.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* One SBC frame is 128 samples per channel at the usual 8-subband, 16-block
 * settings -- 2.9 ms at 44.1 kHz. Sized generously so unusual settings fit. */
#define SBC_MAX_PCM_SAMPLES 512     /* interleaved stereo int16 */

typedef struct {
    uint32_t sample_rate;   /* from the last frame decoded, 0 until then */
    uint8_t  channels;
} sbc_stream_info_t;

/*
 * The outcome of the last decode, kept distinct because the distinction is
 * diagnostic. CRC is the SBC frame's own CRC-8, not the link's CRC-16: a CRC
 * result means the link check passed but the payload it carried was still
 * corrupt, which is a path the link CRC does not cover. Recovered through a
 * getter rather than returned so callers that don't care (the satellite, the
 * classic hub) keep their existing `if (!sbc_decode_frame(...))` test unchanged.
 */
typedef enum {
    SBC_DECODE_OK,    /* decoded; pcm_out holds the samples */
    SBC_DECODE_CRC,   /* OI_CODEC_SBC_CHECKSUM_MISMATCH: the SBC frame's CRC-8 failed */
    SBC_DECODE_ERR,   /* anything else: no syncword, truncated header/body, bad bitpool */
} sbc_decode_result_t;

/* Call once before the first frame, and again to recover after a hard error. */
bool sbc_decoder_init(void);

/*
 * Decode one frame.
 *
 * `in`/`in_len` point at a complete SBC frame; on return `in_consumed` says how
 * many bytes were used, so a caller holding several concatenated frames can
 * advance through them.
 *
 * `pcm_out` receives interleaved 16-bit stereo, `*pcm_samples` the count of
 * int16 values written (frames x channels).
 */
bool sbc_decode_frame(const uint8_t *in, size_t in_len, size_t *in_consumed,
                      int16_t *pcm_out, size_t *pcm_samples);

void sbc_decoder_get_info(sbc_stream_info_t *out);

/*
 * The result of the most recent sbc_decode_frame(); SBC_DECODE_OK before the
 * first call, after sbc_decoder_init(), and after a successful decode. Mirrors
 * sbc_decoder_get_info(): the decoder keeps state across calls, so a getter is
 * how a caller that needs the detail recovers it without every caller paying
 * for an extra out-parameter.
 */
sbc_decode_result_t sbc_decoder_last_result(void);
