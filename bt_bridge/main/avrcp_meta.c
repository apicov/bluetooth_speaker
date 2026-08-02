#include "avrcp_meta.h"

#include <inttypes.h>
#include <string.h>

#include "esp_avrc_api.h"
#include "esp_log.h"

#include "sbc_link.h"
#include "sbc_uart.h"

static const char *TAG = "avrcp";

static link_meta_t s_meta;
static uint8_t s_tl;              /* AVRCP transaction label, wraps at 16 */
static bool s_connected;

/*
 * Last track element id seen.
 *
 * AVRCP notifications are one-shot, so each must be re-registered after it
 * fires -- and registering returns an *interim* response carrying the current
 * value, which fires the callback again. Every real track change therefore
 * arrives twice, a few hundred ms apart.
 *
 * That matters because track_id is meant to be the re-anchor trigger: acting on
 * the duplicate would splice a second time, by which point the new track is
 * already playing and the splice is audible. Comparing the element id tells a
 * genuine change from an echo of the same track.
 */
static uint8_t s_last_elm[8];
static bool s_have_elm;

/* Metadata arrives one attribute per response, so send whatever we have and let
 * later responses fill the rest. A receiver keyed on track_id can tell an
 * update from a new track. */
static void publish(void)
{
    sbc_uart_send_meta(&s_meta);
}

static void request_metadata(void)
{
    esp_avrc_ct_send_metadata_cmd(s_tl++ & 0x0F,
                                  ESP_AVRC_MD_ATTR_TITLE |
                                  ESP_AVRC_MD_ATTR_ARTIST |
                                  ESP_AVRC_MD_ATTR_ALBUM);
}

/* Notifications are one-shot: each must be re-registered after it fires. */
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
            s_have_elm = false;
            register_track_change();
            request_metadata();
        }
        break;

    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        if (rc->change_ntf.event_id == ESP_AVRC_RN_TRACK_CHANGE) {
            const uint8_t *elm = rc->change_ntf.event_parameter.elm_id;
            bool same = s_have_elm && memcmp(elm, s_last_elm, sizeof(s_last_elm)) == 0;
            memcpy(s_last_elm, elm, sizeof(s_last_elm));
            s_have_elm = true;

            if (same) {
                /* Re-registration echo, not a new track. Re-arm and refresh the
                 * text, but do not advance track_id. */
                register_track_change();
                request_metadata();
                break;
            }
            s_meta.track_id++;
            /* Clear rather than keep stale text: a wrong title is worse than
             * none while the new one is being fetched. */
            s_meta.title[0] = s_meta.artist[0] = s_meta.album[0] = '\0';
            ESP_LOGI(TAG, "track change (#%" PRIu32 ")", s_meta.track_id);
            publish();
            register_track_change();      /* one-shot, re-arm it */
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

void avrcp_meta_start(void)
{
    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(avrcp_meta_ct_cb));
    ESP_LOGI(TAG, "AVRCP controller up");
}
