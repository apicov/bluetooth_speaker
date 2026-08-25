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
#include "a2dp_sink_ext_codec_utils.h"

#include "sbc_spi.h"
#include "avrcp_meta.h"
#include "status_led.h"

static const char local_device_name[] = CONFIG_BRIDGE_BT_DEVICE_NAME;

enum {
    BT_APP_EVT_STACK_UP = 0,
};

static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param);

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf);

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

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

            status_led_set_connected(param->conn_stat.state ==
                                     ESP_A2D_CONNECTION_STATE_CONNECTED);

            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                sbc_link_send_vol(avrcp_meta_volume());
            }
        }
        if (event == ESP_A2D_AUDIO_CFG_EVT) {

            const esp_a2d_cie_sbc_t sbc = param->audio_cfg.mcc.cie.sbc_info;

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

        avrcp_meta_start();
        assert(esp_a2d_sink_init() == ESP_OK);

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

        mcc.cie.sbc_info.max_bitpool = 250;
        mcc.cie.sbc_info.min_bitpool = 2;

        esp_a2d_sink_register_stream_endpoint(0, &mcc);
        esp_a2d_sink_register_audio_data_callback(bt_app_a2d_audio_data_cb);

        esp_a2d_sink_get_delay_value();

        esp_bt_gap_get_device_name();

        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }

    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(bredr_app_common_init());

    sbc_link_start();
    status_led_start();

    bt_app_task_start_up();

    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
}
