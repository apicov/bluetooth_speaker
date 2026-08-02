#include "sbc_decoder.h"

#include <string.h>

#include "esp_log.h"

#include "oi_codec_sbc.h"
#include "oi_status.h"

static OI_CODEC_SBC_DECODER_CONTEXT s_ctx;
/* Scratch the codec needs across calls; size is prescribed by the OI headers. */
static uint32_t s_ctx_data[CODEC_DATA_WORDS(2, SBC_CODEC_FAST_FILTER_BUFFERS)];
static sbc_stream_info_t s_info;

bool sbc_decoder_init(void)
{
    memset(&s_info, 0, sizeof(s_info));
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
    OI_UINT32 out_bytes = SBC_MAX_PCM_SAMPLES * sizeof(int16_t);

    OI_STATUS st = OI_CODEC_SBC_DecodeFrame(&s_ctx, &pin, &avail,
                                            (OI_INT16 *)pcm_out, &out_bytes);
    if (st != OI_STATUS_SUCCESS) {
        return false;
    }

    /* The codec advances `pin` and decrements `avail` past what it consumed. */
    *in_consumed = in_len - (size_t)avail;
    *pcm_samples = out_bytes / sizeof(int16_t);

    s_info.sample_rate = s_ctx.common.frameInfo.frequency;
    s_info.channels = s_ctx.common.frameInfo.nrof_channels;
    return true;
}

void sbc_decoder_get_info(sbc_stream_info_t *out)
{
    *out = s_info;
}
