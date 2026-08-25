/**
 * @file sbc_decoder.c
 * @brief The wrapper's whole implementation: reset, decode, and the two
 *        getters that recover state the codec keeps across calls.
 *
 * Everything here is a thin translation into the OI codec's conventions.
 * sbc_decoder.h owns the contract; the codec itself is vendored in oi/ and is
 * not this project's code.
 *
 * Single-threaded by construction: the decoder context below is one static
 * instance, so the caller must decode from one task. Both firmwares do -- the
 * satellite from its play path, the hub from sbc_in's rx task.
 */
#include "sbc_decoder.h"

#include <string.h>

#include "esp_log.h"

#include "oi_codec_sbc.h"
#include "oi_status.h"

/** @brief The codec's own state, carried across frames. */
static OI_CODEC_SBC_DECODER_CONTEXT s_ctx;
/** @brief Scratch the codec needs across calls. The size is prescribed by the
 *         OI headers and must not be guessed at. */
static uint32_t s_ctx_data[CODEC_DATA_WORDS(2, SBC_CODEC_FAST_FILTER_BUFFERS)];
/** @brief Behind sbc_decoder_get_info(). */
static sbc_stream_info_t s_info;
/** @brief Behind sbc_decoder_last_result(). */
static sbc_decode_result_t s_last;

/* Declared in sbc_decoder.h, like the three entry points below it. */
bool sbc_decoder_init(void)
{
    memset(&s_info, 0, sizeof(s_info));
    s_last = SBC_DECODE_OK;
    OI_STATUS st = OI_CODEC_SBC_DecoderReset(&s_ctx, s_ctx_data,
                                             sizeof(s_ctx_data),  /* BYTES, not words */
                                             2,      /* maxChannels */
                                             2,      /* pcmStride: interleaved stereo */
                                             FALSE,  /* enhanced SBC off */
                                             FALSE); /* mSBC off */
    if (st != OI_STATUS_SUCCESS) {
        ESP_LOGE("sbc", "OI_CODEC_SBC_DecoderReset failed: %d", (int)st);
        return false;
    }
    return true;
}

bool sbc_decode_frame(const uint8_t *in, size_t in_len, size_t *in_consumed,
                      int16_t *pcm_out, size_t *pcm_samples)
{
    const OI_BYTE *pin = (const OI_BYTE *)in;
    OI_UINT32 avail = (OI_UINT32)in_len;
    /* The codec is told the buffer's size in BYTES and writes the same field
     * back as the count it produced. */
    OI_UINT32 out_bytes = SBC_MAX_PCM_SAMPLES * sizeof(int16_t);

    OI_STATUS st = OI_CODEC_SBC_DecodeFrame(&s_ctx, &pin, &avail,
                                            (OI_INT16 *)pcm_out, &out_bytes);
    if (st != OI_STATUS_SUCCESS) {
        /* The SBC frame's own CRC-8 is a corruption signal the link CRC-16 does
         * not give, so keep it separate from every other failure. */
        s_last = (st == OI_CODEC_SBC_CHECKSUM_MISMATCH) ? SBC_DECODE_CRC
                                                        : SBC_DECODE_ERR;
        return false;
    }

    /* The codec advances `pin` and decrements `avail` past what it consumed. */
    *in_consumed = in_len - (size_t)avail;
    *pcm_samples = out_bytes / sizeof(int16_t);

    s_info.sample_rate = s_ctx.common.frameInfo.frequency;
    s_info.channels = s_ctx.common.frameInfo.nrof_channels;
    s_last = SBC_DECODE_OK;
    return true;
}

void sbc_decoder_get_info(sbc_stream_info_t *out)
{
    *out = s_info;
}

sbc_decode_result_t sbc_decoder_last_result(void)
{
    return s_last;
}
