/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/**
 * @file main.c
 * @brief The A2DP sink itself: bring Bluetooth up, advertise an SBC endpoint,
 *        and hand every undecoded payload to the SPI link.
 *
 * Derived from the IDF A2DP sink example, whose licence header stands above,
 * and the event plumbing below is still largely its shape: the stack's
 * callbacks run on the Bluetooth task and hand work to a queue, and the shared
 * example components carry the handlers that queue drains.
 *
 * What this project added is small and all in one direction. The audio data
 * callback forwards SBC to the hub instead of decoding it -- this chip has no
 * output and never decodes. AVRCP is brought up (avrcp_meta.h) for metadata and
 * volume, which the example never initialises. And two front-panel LEDs
 * (status_led.h) are fed from the events that already pass through here.
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
#include "a2dp_sink_ext_codec_utils.h"

#include "sbc_spi.h"
#include "avrcp_meta.h"
#include "status_led.h"

/** @brief What the phone lists this bridge as; set on the stack once at
 *         startup, so changing it means a rebuild. */
static const char local_device_name[] = CONFIG_BRIDGE_BT_DEVICE_NAME;

/** @brief Events this file posts to the Bluetooth work queue. */
enum {
    BT_APP_EVT_STACK_UP = 0,   /**< The stack is up; configure it. */
};

/********************************
 * STATIC FUNCTION DECLARATIONS
 *******************************/

/**
 * @brief Device events, passed straight to the shared handler.
 * @param event  Which device event arrived.
 * @param param  Its payload.
 */
static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param);

/**
 * @brief GAP events, passed straight to the shared handler.
 * @param event  Which GAP event arrived.
 * @param param  Its payload.
 */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/**
 * @brief A2DP sink events: dispatched to the work queue, with the two this
 *        project reads taken here on the way past.
 * @param event  Which A2DP event arrived.
 * @param param  Its payload, copied by the dispatch.
 */
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

/**
 * @brief Undecoded audio from the sink, on the Bluetooth task.
 * @param conn_hdl  The connection it arrived on.
 * @param audio_buf  The payload and its length. Owned by the stack.
 */
static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf);

/**
 * @brief Work-queue handler for the events this file posts to itself.
 * @param event    One of the enum above.
 * @param p_param  Its parameter block; unused here.
 */
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
        if (event == ESP_A2D_CONNECTION_STATE_EVT) {
            /* Read here rather than in the work-queue handler: that one lives
             * in a shared example component, and the LED is the only thing this
             * project wants out of the event. */
            status_led_set_connected(param->conn_stat.state ==
                                     ESP_A2D_CONNECTION_STATE_CONNECTED);
            /* Audio starting is the moment a wrong level becomes audible, so
             * re-state it here as well as on the AVRCP connect. The two events
             * are not the same instant and neither reliably precedes the other;
             * sending twice costs a byte and removes the ordering question. */
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                sbc_link_send_vol(avrcp_meta_volume());
            }
        }
        if (event == ESP_A2D_AUDIO_CFG_EVT) {
            /* The codec the phone actually agreed to, and the only place the
             * negotiated ceiling can be read: max_bitpool here is the minimum
             * of what this endpoint advertised and the handset's own limit, so
             * it says whether advertising more could ever buy anything. */
            /* Copied out by value, not taken by pointer: sbc_info sits in a
             * packed union, and taking its address trips
             * -Waddress-of-packed-member. */
            const esp_a2d_cie_sbc_t sbc = param->audio_cfg.mcc.cie.sbc_info;
            /* These are the phone's *selected* configuration -- one bit per
             * field -- rather than the capability mask advertised below, and
             * the bits do not run in the order intuition suggests: mono is the
             * highest channel-mode bit and joint stereo the lowest. Raw hex
             * therefore reads backwards, so decode by name. */
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
        bt_app_work_dispatch(bt_a2d_evt_ext_codec_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf)
{
    /* Undecoded SBC straight out to the hub -- no decode on this chip at all.
     * Both calls copy what they need and return; bt_a2d_audio_data_hdl() then
     * owns the buffer and frees it. */
    status_led_note_audio();
    sbc_link_send(audio_buf->data, audio_buf->data_len);
    bt_a2d_audio_data_hdl(conn_hdl, audio_buf);
}

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event) {

    case BT_APP_EVT_STACK_UP: {
        esp_bt_gap_set_device_name(local_device_name);
        esp_bt_dev_register_callback(bt_app_dev_cb);
        esp_bt_gap_register_callback(bt_app_gap_cb);

        esp_a2d_register_callback(&bt_app_a2d_cb);
        /* Before the sink starts, so that AVRCP is advertised by the time the
         * phone connects and asks for it. */
        avrcp_meta_start();
        assert(esp_a2d_sink_init() == ESP_OK);

        /* Everything SBC allows is advertised. The bridge does not decode, so
         * no combination here costs it anything, and the endpoint should not
         * be the reason a handset has to fall back. */
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
         * The bitpool range, advertised as HEADROOM rather than as a target.
         *
         * The maximum is the codec's own ceiling, and what a phone actually
         * uses is the smaller of this and its own limit -- which the AUDIO_CFG
         * event above reads back. A handset that caps itself lower negotiates
         * lower whatever is advertised here, so a high ceiling costs it
         * nothing; a handset that supports more would use it with no code
         * change.
         *
         * It is not free of consequences elsewhere, and the link is sized for
         * exactly this. SBC_LINK_MAX_PAYLOAD is the payload a full bitpool can
         * produce at any phone MTU, and sbc_spi.c counts anything larger rather
         * than truncating it -- that counter is the tripwire if this ceiling
         * and that one ever fall out of step.
         */
        mcc.cie.sbc_info.max_bitpool = 250;
        mcc.cie.sbc_info.min_bitpool = 2;
        /* One endpoint, SBC only: it is the only codec every A2DP source must
         * implement, and the only one the hub can decode. */
        esp_a2d_sink_register_stream_endpoint(0, &mcc);
        esp_a2d_sink_register_audio_data_callback(bt_app_a2d_audio_data_cb);

        /* Both answer through the callbacks above, and are asked here so that
         * the startup log states what the stack came up with. */
        esp_a2d_sink_get_delay_value();
        esp_bt_gap_get_device_name();

        /* Discoverable and connectable, and left that way: the bridge has no
         * user interface with which to be put into a pairing mode. */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }

    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/*******************************
 * MAIN ENTRY POINT
 ******************************/

/**
 * @brief Bring up the link, the panel, and then Bluetooth.
 *
 * The SPI link starts first, so that it is ready before any audio can arrive;
 * the rest of the Bluetooth configuration happens on the work queue, in
 * bt_av_hdl_stack_evt().
 */
void app_main(void)
{
    ESP_ERROR_CHECK(bredr_app_common_init());

    sbc_link_start();
    status_led_start();

    bt_app_task_start_up();
    /* Device name, connection mode and profiles are set from the work queue
     * rather than here, so they run on the task that owns the stack. */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
}
