/**
 * @file sbc_decoder.h
 * @brief SBC decoding for the dancefloor.
 *
 * Wraps the OI decoder vendored in oi/ so callers deal in "here is an SBC
 * frame, give me PCM" rather than in the codec's own conventions.
 *
 * Why this exists: the bridge forwards SBC frames untouched instead of
 * decoding them, so the inter-chip link and the WiFi stream carry the codec's
 * bitrate rather than the PCM one -- roughly a quarter of the bytes. Each unit
 * decodes at the point of playback instead. Quality is unaffected: it is the
 * same SBC, decoded once, just later.
 *
 * The decoder keeps state across calls, so both the stream format and the last
 * decode's outcome are recovered through getters rather than returned. That
 * keeps the common call site a plain `if (!sbc_decode_frame(...))` and makes
 * every caller that does not need the detail pay nothing for it.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Ceiling on the int16 values one decoded frame can produce.
 *
 * One SBC frame is 128 samples per channel at the usual 8-subband, 16-block
 * settings. Sized generously above that so unusual settings still fit, since
 * the buffer is the caller's and overrunning it is not a failure the codec
 * would report.
 */
#define SBC_MAX_PCM_SAMPLES 512     /* interleaved stereo int16 */

/** @brief What the stream turned out to be, read off the frames as they
 *         decode. See sbc_decoder_get_info(). */
typedef struct {
    uint32_t sample_rate;   /**< From the last frame decoded; 0 until then. */
    uint8_t  channels;      /**< Likewise: 1 or 2. */
} sbc_stream_info_t;

/**
 * @brief The outcome of the last decode, kept distinct because the distinction
 *        is diagnostic.
 *
 * CRC here is the SBC frame's own CRC-8, not the link's CRC-16. A CRC result
 * therefore means the link check PASSED and the payload it carried was still
 * corrupt -- a path the link CRC does not cover, and the reason this is not
 * folded into the general error.
 */
typedef enum {
    SBC_DECODE_OK,    /**< Decoded; pcm_out holds the samples. */
    SBC_DECODE_CRC,   /**< OI_CODEC_SBC_CHECKSUM_MISMATCH: the frame's CRC-8 failed. */
    SBC_DECODE_ERR,   /**< Anything else: no syncword, truncated header or body,
                       *   bad bitpool. */
} sbc_decode_result_t;

/**
 * @brief Reset the codec. Call once before the first frame, and again to
 *        recover after a hard error.
 *
 * @return false if the codec refused the reset, which is a build or
 *         configuration fault rather than a stream one.
 */
bool sbc_decoder_init(void);

/**
 * @brief Decode one frame.
 *
 * @param in                A complete SBC frame.
 * @param in_len            Bytes available at @p in.
 * @param[out] in_consumed  Bytes actually used, so a caller holding several
 *                          concatenated frames can advance through them.
 * @param[out] pcm_out      Interleaved 16-bit stereo; must have room for
 *                          SBC_MAX_PCM_SAMPLES values.
 * @param[out] pcm_samples  Count of int16 values written (frames x channels).
 * @return true on success. On false nothing is written to @p pcm_out and
 *         sbc_decoder_last_result() says whether the frame failed its CRC or
 *         was malformed.
 */
bool sbc_decode_frame(const uint8_t *in, size_t in_len, size_t *in_consumed,
                      int16_t *pcm_out, size_t *pcm_samples);

/**
 * @brief What the stream has turned out to be.
 *
 * @param[out] out  Filled from the last frame decoded; zeroed by
 *                  sbc_decoder_init() and so all-zero until the first success.
 */
void sbc_decoder_get_info(sbc_stream_info_t *out);

/**
 * @brief The result of the most recent sbc_decode_frame().
 *
 * @return SBC_DECODE_OK before the first call, after sbc_decoder_init(), and
 *         after any successful decode; otherwise which way the last one failed.
 */
sbc_decode_result_t sbc_decoder_last_result(void);
