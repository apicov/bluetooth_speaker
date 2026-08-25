#include "avrcp_meta.h"

#include <inttypes.h>
#include <string.h>

#include "esp_avrc_api.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sbc_link.h"
#include "sbc_spi.h"

static const char *TAG = "avrcp";

static link_meta_t s_meta;
static uint8_t s_tl;
static bool s_connected;

#define TRACK_CHANGE_ECHO_US 1500000

static int64_t s_last_change_us;

static void publish(void)
{
    sbc_link_send_meta(&s_meta);
}

static void request_metadata(void)
{
    esp_avrc_ct_send_metadata_cmd(s_tl++ & 0x0F,
                                  ESP_AVRC_MD_ATTR_TITLE |
                                  ESP_AVRC_MD_ATTR_ARTIST |
                                  ESP_AVRC_MD_ATTR_ALBUM);
}

static void register_track_change(void)
{
    esp_avrc_ct_send_register_notification_cmd(s_tl++ & 0x0F,
                                               ESP_AVRC_RN_TRACK_CHANGE, 0);
}

static void copy_text(char *dst, const uint8_t *src, int len)
{
    if (len < 0) {
        len = 0;
    }
    if (len > META_TEXT_LEN - 1) {
        len = META_TEXT_LEN - 1;
    }
    memcpy(dst, src, len);
    dst[len] = '\0';
}

void avrcp_meta_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *rc)
{

    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        s_connected = rc->conn_stat.connected;
        ESP_LOGI(TAG, "AVRCP %s", s_connected ? "connected" : "disconnected");
        if (s_connected) {
            memset(&s_meta, 0, sizeof(s_meta));
            s_last_change_us = 0;
            register_track_change();
            request_metadata();
        }
        break;

    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        if (rc->change_ntf.event_id == ESP_AVRC_RN_TRACK_CHANGE) {
            int64_t now = esp_timer_get_time();
            bool echo = s_last_change_us != 0 &&
                        (now - s_last_change_us) < TRACK_CHANGE_ECHO_US;
            s_last_change_us = now;

            if (echo) {

                register_track_change();
                request_metadata();
                break;
            }
            s_meta.track_id++;

            s_meta.title[0] = s_meta.artist[0] = s_meta.album[0] = '\0';
            ESP_LOGI(TAG, "track change (#%" PRIu32 ")", s_meta.track_id);
            publish();
            register_track_change();
            request_metadata();
        }
        break;

    case ESP_AVRC_CT_METADATA_RSP_EVT:
        switch (rc->meta_rsp.attr_id) {
        case ESP_AVRC_MD_ATTR_TITLE:
            copy_text(s_meta.title, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
            break;
        case ESP_AVRC_MD_ATTR_ARTIST:
            copy_text(s_meta.artist, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
            break;
        case ESP_AVRC_MD_ATTR_ALBUM:
            copy_text(s_meta.album, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
            break;
        default:
            return;
        }
        ESP_LOGI(TAG, "#%" PRIu32 " \"%s\" - %s [%s]",
                 s_meta.track_id, s_meta.title, s_meta.artist, s_meta.album);
        publish();
        break;

    default:
        break;
    }
}

static uint8_t s_volume = AUDIO_VOL_MAX;

#define VOL_HEARTBEAT_US 5000000

static void vol_heartbeat_cb(void *arg)
{
    (void)arg;
    sbc_link_send_vol(s_volume);
}

static void vol_heartbeat_start(void)
{
    const esp_timer_create_args_t args = {
        .callback = vol_heartbeat_cb,
        .name = "volhb",
    };
    esp_timer_handle_t h;
    ESP_ERROR_CHECK(esp_timer_create(&args, &h));
    ESP_ERROR_CHECK(esp_timer_start_periodic(h, VOL_HEARTBEAT_US));
}

uint8_t avrcp_meta_volume(void)
{
    return s_volume;
}

void avrcp_meta_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *rc)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "AVRCP target %s",
                 rc->conn_stat.connected ? "connected" : "disconnected");

        if (rc->conn_stat.connected) {
            sbc_link_send_vol(s_volume);
        }
        break;

    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        s_volume = rc->set_abs_vol.volume > AUDIO_VOL_MAX
                 ? AUDIO_VOL_MAX : rc->set_abs_vol.volume;
        ESP_LOGI(TAG, "volume %u/%d", s_volume, AUDIO_VOL_MAX);
        sbc_link_send_vol(s_volume);
        break;

    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:

        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            esp_avrc_rn_param_t rn = { .volume = s_volume };
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE,
                                    ESP_AVRC_RN_RSP_INTERIM, &rn);
        }
        break;

    default:
        break;
    }
}

void avrcp_meta_start(void)
{
    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(avrcp_meta_ct_cb));

    ESP_ERROR_CHECK(esp_avrc_tg_init());
    ESP_ERROR_CHECK(esp_avrc_tg_register_callback(avrcp_meta_tg_cb));

    esp_avrc_rn_evt_cap_mask_t evt_set = { 0 };
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set,
                                       ESP_AVRC_RN_VOLUME_CHANGE);
    ESP_ERROR_CHECK(esp_avrc_tg_set_rn_evt_cap(&evt_set));

    vol_heartbeat_start();

    ESP_LOGI(TAG, "AVRCP controller and target up, volume %u/%d, re-stated every %d s",
             s_volume, AUDIO_VOL_MAX, (int)(VOL_HEARTBEAT_US / 1000000));
}
