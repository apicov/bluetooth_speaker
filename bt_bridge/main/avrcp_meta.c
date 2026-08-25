/**
 * @file avrcp_meta.c
 * @brief The AVRCP controller and target callbacks, and the volume heartbeat.
 *        Declared in avrcp_meta.h, which is where the reason for each half is.
 *
 * Metadata is accumulated into one link_meta_t and republished whenever any
 * part of it changes; volume is forwarded as it arrives and re-stated on a
 * timer. Nothing here is applied locally -- this unit has no output.
 */
#include "avrcp_meta.h"

#include <inttypes.h>
#include <string.h>

#include "esp_avrc_api.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sbc_link.h"
#include "sbc_spi.h"

/** @brief ESP_LOG tag for both AVRCP halves. */
static const char *TAG = "avrcp";

/**
 * @brief The metadata as it stands, and what publish() sends.
 *
 * Kept whole rather than assembled per response, because attributes arrive one
 * at a time and the hub wants a complete record with each update.
 */
static link_meta_t s_meta;
/** @brief AVRCP transaction label for outgoing commands; four bits, so it
 *         wraps at sixteen. */
static uint8_t s_tl;
/** @brief Whether the controller currently has a peer. */
static bool s_connected;

/**
 * @brief How close behind a track-change notification a second one still counts
 *        as the same change.
 *
 * AVRCP notifications are one-shot, so each must be re-registered after it
 * fires -- and registering returns an *interim* response carrying the current
 * value, which fires the callback again. Every real track change therefore
 * arrives twice.
 *
 * That matters because link_meta_t::track_id drives the re-anchor downstream:
 * acting on the duplicate would splice a second time, by which point the new
 * track is already playing and the splice is audible.
 *
 * Deduplicated by TIME, not by track element id. Many phones report the element
 * id as zero -- "whatever is playing now" -- so comparing it matches every time
 * and track_id never advances at all. The echo follows its own notification
 * within a fraction of a second, while genuine track changes are a song apart,
 * so this window sits comfortably between the two and is not delicate.
 */
#define TRACK_CHANGE_ECHO_US 1500000

/** @brief When the last track-change notification arrived, in esp_timer
 *         microseconds; zero before the first one. */
static int64_t s_last_change_us;

/**
 * @brief Send the metadata as it currently stands.
 *
 * Called after every change rather than once the record is complete: attributes
 * arrive one response at a time, and a receiver keyed on link_meta_t::track_id
 * can tell an update from a new track without comparing strings.
 */
static void publish(void)
{
    sbc_link_send_meta(&s_meta);
}

/** @brief Ask the phone for title, artist and album in one command. */
static void request_metadata(void)
{
    esp_avrc_ct_send_metadata_cmd(s_tl++ & 0x0F,
                                  ESP_AVRC_MD_ATTR_TITLE |
                                  ESP_AVRC_MD_ATTR_ARTIST |
                                  ESP_AVRC_MD_ATTR_ALBUM);
}

/** @brief Arm the track-change notification. One-shot, so every path that
 *         handles one calls this again. */
static void register_track_change(void)
{
    esp_avrc_ct_send_register_notification_cmd(s_tl++ & 0x0F,
                                               ESP_AVRC_RN_TRACK_CHANGE, 0);
}

/**
 * @brief Copy one AVRCP text attribute into a fixed metadata field.
 *
 * Truncates rather than refuses: a shortened title is a better failure than a
 * dropped update, and the field width is the protocol's business rather than
 * the phone's.
 *
 * @param dst  Field of META_TEXT_LEN bytes; always left NUL-terminated.
 * @param src  Attribute text from the stack, which is not terminated.
 * @param len  Its length as reported, treated as zero if negative.
 */
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

/* Declared in avrcp_meta.h. */
void avrcp_meta_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *rc)
{

    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        s_connected = rc->conn_stat.connected;
        ESP_LOGI(TAG, "AVRCP %s", s_connected ? "connected" : "disconnected");
        /* A new peer is a new world: drop whatever the last one was playing,
         * forget its timing, and start the notification and the query again. */
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
                /* The re-registration echo, not a new track. Re-arm and refresh
                 * the text, but do not advance track_id. */
                register_track_change();
                request_metadata();
                break;
            }
            s_meta.track_id++;
            /* Clear rather than keep stale text: a wrong title is worse than
             * none while the new one is being fetched. */
            s_meta.title[0] = s_meta.artist[0] = s_meta.album[0] = '\0';
            ESP_LOGI(TAG, "track change (#%" PRIu32 ")", s_meta.track_id);
            /* Published before the text arrives, because the boundary is the
             * time-critical part and the title is not. */
            publish();
            register_track_change();
            request_metadata();
        }
        break;

    case ESP_AVRC_CT_METADATA_RSP_EVT:
        /* One attribute per response. Anything not asked for is ignored
         * without republishing -- nothing changed. */
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

/** @brief The level the phone last stated. Full scale until it states one,
 *         which here is a value to forward rather than a loudness. */
static uint8_t s_volume = AUDIO_VOL_MAX;

/**
 * @brief How often the current volume is re-stated on the link.
 *
 * The bridge is the only unit that ever hears the phone, so it is the only one
 * that can repair a hub which has forgotten the level. There are three ways to
 * forget, all of them silent: the hub reboots and comes up at its own default,
 * with nothing to correct it until somebody touches the slider;
 * sbc_link_send_vol() enqueues with a zero timeout, so a full ring drops the
 * change and counts it; or the frame arrives with a bad CRC and is refused.
 *
 * A heartbeat covers all three without the bridge having to detect any of them,
 * which is the point -- detecting a hub reboot from this side means a protocol,
 * and that protocol would be worth more than the byte it protects.
 *
 * One padded frame at this interval is a negligible fraction of the link's
 * time. And since a speaker stays silent until it has been told a level, the
 * interval bounds a silence rather than a loudness: the worst it can cost is a
 * few seconds of quiet, never a blast.
 */
#define VOL_HEARTBEAT_US 5000000

/**
 * @brief Heartbeat timer callback: re-state the volume.
 * @param arg  Unused.
 */
static void vol_heartbeat_cb(void *arg)
{
    (void)arg;
    sbc_link_send_vol(s_volume);
}

/**
 * @brief Create and start the heartbeat timer.
 *
 * The handle is not kept: the timer runs for the life of the unit, and there is
 * no state in which stopping it would be right.
 */
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

/* Declared in avrcp_meta.h. */
uint8_t avrcp_meta_volume(void)
{
    return s_volume;
}

/* Declared in avrcp_meta.h. */
void avrcp_meta_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *rc)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "AVRCP target %s",
                 rc->conn_stat.connected ? "connected" : "disconnected");
        /* Best knowledge at that instant, sent without waiting to be asked. The
         * phone usually overwrites it with an absolute volume almost at
         * once, and a handset that only sends the level when it changes
         * never would. */
        if (rc->conn_stat.connected) {
            sbc_link_send_vol(s_volume);
        }
        break;

    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        /* Clamped rather than trusted: the field is wider than the range AVRCP
         * defines for it, and the speakers scale by it. */
        s_volume = rc->set_abs_vol.volume > AUDIO_VOL_MAX
                 ? AUDIO_VOL_MAX : rc->set_abs_vol.volume;
        ESP_LOGI(TAG, "volume %u/%d", s_volume, AUDIO_VOL_MAX);
        sbc_link_send_vol(s_volume);
        break;

    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
        /*
         * Answered INTERIM and never followed by CHANGED, deliberately.
         *
         * The notification exists so that a sink with its own volume control --
         * a knob, a button -- can tell the phone the user turned it. Nothing
         * here has one: volume only ever arrives FROM the phone, so there is
         * never a change to report back, and a CHANGED response would only echo
         * what the controller had just sent. An interim response carrying the
         * current value is the whole of this target's obligation.
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

/* Declared in avrcp_meta.h. */
void avrcp_meta_start(void)
{
    ESP_ERROR_CHECK(esp_avrc_ct_init());
    ESP_ERROR_CHECK(esp_avrc_ct_register_callback(avrcp_meta_ct_cb));

    ESP_ERROR_CHECK(esp_avrc_tg_init());
    ESP_ERROR_CHECK(esp_avrc_tg_register_callback(avrcp_meta_tg_cb));

    /* Declare which notifications this target supports. Volume change is the
     * one that matters: without it in the capability set the phone never sends
     * an absolute volume and goes on scaling the audio itself. */
    esp_avrc_rn_evt_cap_mask_t evt_set = { 0 };
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set,
                                       ESP_AVRC_RN_VOLUME_CHANGE);
    ESP_ERROR_CHECK(esp_avrc_tg_set_rn_evt_cap(&evt_set));

    vol_heartbeat_start();

    ESP_LOGI(TAG, "AVRCP controller and target up, volume %u/%d, re-stated every %d s",
             s_volume, AUDIO_VOL_MAX, (int)(VOL_HEARTBEAT_US / 1000000));
}
