/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"

#include "bt_app_core_utils.h"
#include "bredr_app_common_utils.h"
#include "a2dp_sink_common_utils.h"
#include "a2dp_utils_tags.h"
#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
#include "a2dp_sink_int_codec_utils.h"
#else
#include "a2dp_sink_ext_codec_utils.h"
#endif

#include "sbc_spi.h"
#include "avrcp_meta.h"

/* device name */
static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;

/* event for stack up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/********************************
 * STATIC FUNCTION DECLARATIONS
 *******************************/

/* Device callback function */
static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param);

/* GAP callback function */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/* callback function for A2DP sink */
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
/* callback function for A2DP sink audio data stream */
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
#else
/* callback function for A2DP sink undecoded audio data */
static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf);
#endif

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/

static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param)
{
    bredr_app_dev_evt_def_hdl(event, param);
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    bredr_app_gap_evt_def_hdl(event, param);
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        bt_app_work_dispatch(bt_a2d_evt_def_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL);
        break;
    }
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_SEP_REG_STATE_EVT: {
        if (event == ESP_A2D_AUDIO_CFG_EVT) {
            /* The codec the phone actually agreed to. max_bitpool here is the
             * negotiated ceiling -- min(what we advertised, the phone's own max)
             * -- so with our max at 250 it reads back the phone's real limit.
             * That is the number that decides whether raising max_bitpool can
             * ever buy quality, or whether the phone caps it regardless. */
            /* Copy out by value, not by pointer: sbc_info sits in a packed
             * union, and &member trips -Waddress-of-packed-member. */
            const esp_a2d_cie_sbc_t sbc = param->audio_cfg.mcc.cie.sbc_info;
            /* These are the phone's *selected* config -- one bit per field, not
             * the capability mask we advertise -- so decode by name rather than
             * raw hex. (0x01 ch_mode is joint stereo, not mono: in
             * esp_a2dp_api.h MONO=0x8, JOINT_STEREO=0x1.) */
            const char *sf = sbc.samp_freq & ESP_A2D_SBC_CIE_SF_48K ? "48k"
                           : sbc.samp_freq & ESP_A2D_SBC_CIE_SF_44K ? "44.1k"
                           : sbc.samp_freq & ESP_A2D_SBC_CIE_SF_32K ? "32k" : "16k";
            const char *cm = sbc.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO         ? "mono"
                           : sbc.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL ? "dual"
                           : sbc.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_STEREO       ? "stereo"
                           : "joint";
            const char *bl = sbc.block_len & ESP_A2D_SBC_CIE_BLOCK_LEN_16 ? "16"
                           : sbc.block_len & ESP_A2D_SBC_CIE_BLOCK_LEN_12 ? "12"
                           : sbc.block_len & ESP_A2D_SBC_CIE_BLOCK_LEN_8  ? "8"  : "4";
            const char *ns = sbc.num_subbands & ESP_A2D_SBC_CIE_NUM_SUBBANDS_8 ? "8" : "4";
            const char *am = sbc.alloc_mthd & ESP_A2D_SBC_CIE_ALLOC_MTHD_SNR ? "snr" : "loudness";
            ESP_LOGI(BT_AV_TAG, "audio cfg: %s %s | block %s subbands %s %s | bitpool %u..%u",
                     sf, cm, bl, ns, am, sbc.min_bitpool, sbc.max_bitpool);
        }
#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
        bt_app_work_dispatch(bt_a2d_evt_int_codec_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL);
#else
        bt_app_work_dispatch(bt_a2d_evt_ext_codec_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL);
#endif
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    bt_a2d_data_hdl(data, len);
}
#else
static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf)
{
    /* Undecoded SBC straight out to the hub -- no decode on this chip at all.
     * bt_a2d_audio_data_hdl() owns the buffer and frees it. */
    sbc_link_send(audio_buf->data, audio_buf->data_len);
    bt_a2d_audio_data_hdl(conn_hdl, audio_buf);
}
#endif

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event) {
    /* when do the stack up, this event comes */
    case BT_APP_EVT_STACK_UP: {
        esp_bt_gap_set_device_name(local_device_name);
        esp_bt_dev_register_callback(bt_app_dev_cb);
        esp_bt_gap_register_callback(bt_app_gap_cb);

        esp_a2d_register_callback(&bt_app_a2d_cb);
        /* Before A2DP starts, so the phone sees AVRCP advertised on connect --
         * this is what was refusing its PSM 23 request. */
        avrcp_meta_start();
        assert(esp_a2d_sink_init() == ESP_OK);

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
#else
        esp_a2d_mcc_t mcc = {0};
        mcc.type = ESP_A2D_MCT_SBC;
        mcc.cie.sbc_info.samp_freq = ESP_A2D_SBC_CIE_SF_16K |
                                     ESP_A2D_SBC_CIE_SF_32K |
                                     ESP_A2D_SBC_CIE_SF_44K |
                                     ESP_A2D_SBC_CIE_SF_48K;
        mcc.cie.sbc_info.ch_mode = ESP_A2D_SBC_CIE_CH_MODE_MONO |
                                   ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL |
                                   ESP_A2D_SBC_CIE_CH_MODE_STEREO |
                                   ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
        mcc.cie.sbc_info.block_len = ESP_A2D_SBC_CIE_BLOCK_LEN_4 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_8 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_12 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_16;
        mcc.cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_4 | ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
        mcc.cie.sbc_info.alloc_mthd = ESP_A2D_SBC_CIE_ALLOC_MTHD_SNR | ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
        /*
         * The bitpool was a WIRE budget on the UART: 53 was what 500 kbaud could
         * carry, the standard A2DP "high quality" value. SPI cleared the wire
         * bandwidth, so 250 (the codec's own SBC_MAX_BITPOOL) was tried with the
         * payload ceilings at 2048 -- a 2060-byte frame.
         *
         * Two things bound it, and neither is the wire's raw capacity:
         *
         *   The frame on the jumpers. The 2060-byte transfer fails at 10 MHz --
         *   the hub's sbc_in reports `short`/`gaps` while the bridge stays clean
         *   -- but is clean at 5 MHz (slower edges, ~3.3 ms/transfer against a
         *   ~20 ms A2DP cadence). The smaller 1036-byte frame was clean at 10
         *   MHz; the clock is a wire-reliability knob, not a quality one.
         *
         *   The source. The AUDIO_CFG event reads the negotiated codec back, and
         *   this phone returns bitpool 2..53 regardless of what we advertise
         *   (the `audio cfg:` log). 53 is the handset's own ceiling, so raising
         *   our max bought nothing for it -- the quality improvement heard was
         *   the 5 MHz wire fix, never the bitpool.
         *
         * 250 is kept as HEADROOM. It costs this phone nothing -- it negotiates
         * 53 either way -- and a handset that supports more would use it with no
         * code change. The `payload past the link ceiling` counter (s_oversize)
         * is the tripwire if a packet ever outgrows 2048. See docs/sbc-link.md.
         */
        mcc.cie.sbc_info.max_bitpool = 250;
        mcc.cie.sbc_info.min_bitpool = 2;
        /* register stream end point, only support SBC currently */
        esp_a2d_sink_register_stream_endpoint(0, &mcc);
        esp_a2d_sink_register_audio_data_callback(bt_app_a2d_audio_data_cb);
#endif

        /* Get the default value of the delay value */
        esp_a2d_sink_get_delay_value();
        /* Get local device name */
        esp_bt_gap_get_device_name();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/*******************************
 * MAIN ENTRY POINT
 ******************************/

void app_main(void)
{
    ESP_ERROR_CHECK(bredr_app_common_init());

    sbc_link_start();

    bt_app_task_start_up();
    /* bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
}
