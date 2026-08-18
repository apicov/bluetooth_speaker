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
static uint8_t s_tl;              /* AVRCP transaction label, wraps at 16 */
static bool s_connected;

/*
 * AVRCP notifications are one-shot, so each must be re-registered after it
 * fires -- and registering returns an *interim* response carrying the current
 * value, which fires the callback again. Every real track change therefore
 * arrives twice.
 *
 * That matters because track_id drives the re-anchor: acting on the duplicate
 * would splice a second time, by which point the new track is already playing
 * and the splice is audible.
 *
 * Deduplicated by TIME, not by track element id. Many phones report the element
 * id as zero -- "whatever is playing now" -- so comparing it matched every time
 * and track_id never advanced at all. Measured, the echo arrives 120-480 ms
 * after the real notification while genuine changes are tens of seconds apart,
 * so a window comfortably between the two separates them.
 */
#define TRACK_CHANGE_ECHO_US 1500000

static int64_t s_last_change_us;

/* Metadata arrives one attribute per response, so send whatever we have and let
 * later responses fill the rest. A receiver keyed on track_id can tell an
 * update from a new track. */
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

/*
 * The TARGET half, which is what makes the phone's volume slider a control
 * command instead of an attenuation applied to the audio.
 *
 * Without this the handset has nowhere to send volume, so it scales the PCM
 * before the SBC encoder and every step down is resolution destroyed before the
 * audio reaches the air. With it, the phone transmits at full scale and tells us
 * how loud to play; the speakers honour it at their own output, in
 * audio_apply_volume().
 *
 * This unit has no DAC of its own -- it is a bridge -- so it does not act on the
 * volume at all. It forwards it and the speakers apply it.
 */
static uint8_t s_volume = AUDIO_VOL_MAX;   /* unity until the phone says otherwise */

/*
 * Re-state the level on a timer, forever.
 *
 * The bridge is the only unit that ever hears the phone, so it is the only unit
 * that can repair a hub which has forgotten. And there were three ways to
 * forget, all of them silent:
 *
 *   - the hub reboots and comes up at its own default, with nothing to correct
 *     it until somebody touches the slider;
 *   - sbc_link_send_vol() enqueues with a zero timeout, so a full ring drops the
 *     change and counts it as s_dropped;
 *   - the frame arrives with a bad CRC and sbc_in.c refuses it.
 *
 * A heartbeat covers all three without the bridge having to detect any of them,
 * which is the point -- detecting a hub reboot from this side means a protocol,
 * and the protocol would be worth more than the byte it protects.
 *
 * One padded SPI frame per five seconds is 3.3 ms of link time at 5 MHz, 0.07%.
 * And with the speakers staying silent until they are told a level, five seconds
 * is a silence budget rather than a loudness one: the failure it bounds is a
 * quiet floor, not a blast.
 */
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

/* What the phone last said, for the paths that re-state it without a new
 * command to act on -- A2DP connect, in main.c. */
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
        /* Best knowledge at that instant, sent without waiting to be asked. The
         * phone usually overwrites it with SET_ABSOLUTE_VOLUME within a second,
         * and when it does not -- a handset that only sends the level on a
         * change -- this is the only thing that would have. */
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
        /*
         * Answered INTERIM and never followed by CHANGED, deliberately.
         *
         * The notification exists so a sink with its own volume control -- a
         * knob, a button -- can tell the phone the user turned it. Nothing here
         * has one: volume only ever arrives FROM the phone, so there is never a
         * change to report back and a CHANGED would only ever be an echo of
         * what the controller just sent. An interim response with the current
         * value is the whole of this target's obligation.
         */
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

    /* Declare which notifications this target supports. Volume change is the
     * one that matters; without it in the capability set the phone never sends
     * SET_ABSOLUTE_VOLUME and keeps scaling the audio itself. */
    esp_avrc_rn_evt_cap_mask_t evt_set = { 0 };
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set,
                                       ESP_AVRC_RN_VOLUME_CHANGE);
    ESP_ERROR_CHECK(esp_avrc_tg_set_rn_evt_cap(&evt_set));

    vol_heartbeat_start();

    ESP_LOGI(TAG, "AVRCP controller and target up, volume %u/%d, re-stated every %d s",
             s_volume, AUDIO_VOL_MAX, (int)(VOL_HEARTBEAT_US / 1000000));
}
