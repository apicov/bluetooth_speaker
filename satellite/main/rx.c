#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"

#include "audio_out.h"
#include "sync_proto.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

static uint32_t expect_seq;
static bool have_seq;

static int64_t s_prev_audio_at;
static uint32_t s_burst_run;

#define PLC_TAIL_FRAMES 128
#define PLC_FADE_FRAMES 128
static int16_t s_plc_tail[PLC_TAIL_FRAMES * AUDIO_CHANNELS];
static uint32_t s_plc_have;
static uint32_t s_plc_fade_in;

static void plc_note(const int16_t *pcm, uint32_t frames)
{
    if (frames == 0) {
        return;
    }
    const uint32_t take = frames > PLC_TAIL_FRAMES ? PLC_TAIL_FRAMES : frames;
    memcpy(s_plc_tail, pcm + (frames - take) * AUDIO_CHANNELS,
           (size_t)take * AUDIO_CHANNELS * sizeof(int16_t));
    s_plc_have = take;
}

static bool anchor_stream(const audio_msg_t *msg, int64_t offset, bool by_tsf)
{

    if (!by_tsf && !sync_est_settled(&est)) {
        static bool told;
        if (!told) {
            told = true;
            ESP_LOGI(TAG, "holding playback until the clock estimate settles");
        }
        return false;
    }

    if (resync_request) {
        return false;
    }

    const int64_t now_local = esp_timer_get_time();
    const int64_t start_local = sync_to_local(msg->play_at, offset);
    static int64_t refuse_since;
    static int64_t last_anchor;

    if (start_local - now_local < ANCHOR_MIN_LEAD_US ||
        (last_anchor && now_local - last_anchor < ANCHOR_MIN_INTERVAL_US)) {
        const bool late = start_local - now_local < ANCHOR_MIN_LEAD_US;
        if (late) n_anchor_late++; else n_anchor_soon++;
        if (refuse_since == 0) {
            refuse_since = now_local;
        }
        if (now_local - refuse_since < ANCHOR_GIVE_UP_US) {
            return false;
        }
        ESP_LOGE(TAG, "no anchorable packet for %lld ms (lead %+lld ms) -- "
                      "anchoring on a bad one rather than staying silent",
                 (now_local - refuse_since) / 1000,
                 (start_local - now_local) / 1000);
        anchor_provisional = true;
    } else {
        anchor_provisional = false;
    }
    refuse_since = 0;
    last_anchor = now_local;
    anchor_at = now_local;
    resync_request = false;

    stream_rate = msg->sample_rate ? msg->sample_rate : 44100;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

    visualiser_set_rate(stream_rate);

    visualiser_flush();
#endif
    sbc_decoder_init();

    s_plc_have = 0;
    s_plc_fade_in = 0;
    stream_start_local = start_local;
    stream_offset = offset;
    offset_slew_last = 0;

    samples_in = 0;
    marker_sample = -1;
    phase_head = phase_tail = 0;
    phase_valid = false;
    restart_pos = -1;
    expect_seq = msg->seq;
    have_seq = true;
    xStreamBufferReset(ring);
    n_reanchors++;
    if (by_tsf) n_tsf_used++; else n_tsf_fallback++;
    ESP_LOGI(TAG, "stream start: play_at %lld -> local %lld (in %lld ms) [%s]",
             msg->play_at, stream_start_local,
             (stream_start_local - esp_timer_get_time()) / 1000,
             by_tsf ? "TSF" : "probe estimator");

    if (rejoined_at) {
        const int64_t age = est_newest_at ? (now_local - est_newest_at) / 1000 : -1;
        ESP_LOGW(TAG, "first anchor after a rejoin: %s, %lld ms since the "
                      "rejoin, newest probe %lld ms old",
                 by_tsf ? "TSF" : "PROBE ESTIMATOR",
                 (now_local - rejoined_at) / 1000, age);
        rejoined_at = 0;
    }
    return true;
}

static bool upgrade_provisional_anchor(const audio_msg_t *msg, int64_t offset)
{
    if (anchor_provisional &&
        sync_to_local(msg->play_at, offset) - esp_timer_get_time() >= ANCHOR_MIN_LEAD_US) {
        anchor_provisional = false;
        n_anchor_upgrades++;
        resync_request = true;
        have_seq = false;
        return true;
    }
    return false;
}

static uint32_t fill_gap_silence(uint32_t want_frames)
{
    static const int16_t silence[128 * AUDIO_CHANNELS] = {0};
    uint32_t filled = 0;

    static int16_t conceal[128 * AUDIO_CHANNELS];
    uint32_t faded = 0;

    while (filled < want_frames) {
        uint32_t left = want_frames - filled;
        uint32_t n = left > 128 ? 128 : left;
        const int16_t *src = silence;

        if (s_plc_have && faded < PLC_FADE_FRAMES) {
            for (uint32_t i = 0; i < n; i++) {
                const uint32_t pos = faded + i;

                const int32_t gain = pos < PLC_FADE_FRAMES
                                   ? (int32_t)(PLC_FADE_FRAMES - pos) : 0;
                const int16_t *s = &s_plc_tail[(pos % s_plc_have) * AUDIO_CHANNELS];
                for (int c = 0; c < AUDIO_CHANNELS; c++) {
                    conceal[i * AUDIO_CHANNELS + c] =
                        (int16_t)((int32_t)s[c] * gain / (int32_t)PLC_FADE_FRAMES);
                }
            }
            src = conceal;
        }

        size_t want = (size_t)n * AUDIO_CHANNELS * sizeof(int16_t);
        size_t sent = xStreamBufferSend(ring, src, want, 0);
        uint32_t got = (uint32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
        filled += got;
        faded += got;
        samples_in += (int32_t)got;
        if (sent < want) {
            break;
        }
    }

    s_plc_fade_in = PLC_FADE_FRAMES;
    return filled;
}

static bool fill_gap(uint32_t missing, uint32_t per_frames)
{
    const uint32_t frames_missing = missing * per_frames;

    n_gap_frames += frames_missing;
    {

        if (frames_missing > (uint32_t)((uint64_t)GAP_RESYNC_MS * stream_rate / 1000)) {
            n_gap_resyncs++;
            resync_request = true;
            have_seq = false;
            return false;
        }

        size_t room = xStreamBufferSpacesAvailable(ring);
        uint32_t can_take = (uint32_t)(room / (AUDIO_CHANNELS * sizeof(int16_t)));
        if (can_take < frames_missing) {
            n_gap_short++;
            n_gap_short_frames += frames_missing - can_take;
            n_gap_short_resyncs++;
            resync_request = true;
            have_seq = false;
            return false;
        }

        uint32_t filled = 0;
        for (uint32_t i = 0; i < missing && filled < frames_missing; i++) {
            uint32_t per = frames_missing - filled;
            if (per > per_frames) {
                per = per_frames;
            }
            uint32_t got = fill_gap_silence(per);
            filled += got;
            if (got < per) {

                n_gap_short++;
                n_gap_short_frames += per - got;
                n_gap_short_resyncs++;
                resync_request = true;
                have_seq = false;
                return false;
            }
        }
    }
    return true;
}

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
static bool fec_hold_begin(const audio_msg_t *msg, uint32_t missing);
#endif

static bool absorb_sequence_gap(const audio_msg_t *msg)
{
    if (have_seq && msg->seq > expect_seq) {
        const uint32_t missing = msg->seq - expect_seq;
        n_gaps++;

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
        if (fec_hold_begin(msg, missing)) {
            return false;
        }

        n_fec_lost++;
#endif
        return fill_gap(missing, msg->frames);
    } else if (have_seq && msg->seq < expect_seq) {

        n_seq_dropped++;
        return false;
    }
    return true;
}

static void record_packet_positions(const audio_msg_t *msg)
{

    if (msg->marker && marker_sample < 0) {
        marker_sample = samples_in;
    }

    if (msg->restart && restart_pos < 0) {
        restart_pos = samples_in;
    }

    uint32_t next = (phase_head + 1) % PHASE_Q_LEN;
    if (next != phase_tail) {
        phase_q[phase_head].pos = samples_in;
        phase_q[phase_head].play_at = msg->play_at;
        phase_head = next;
    } else {
        n_phase_drop++;
    }
}

static void decode_into_ring(const audio_msg_t *msg)
{
    static int16_t pcm[SBC_MAX_PCM_SAMPLES];

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    uint32_t pkt_frames = 0;
#endif
    size_t off = 0;
    while (off < msg->payload_len) {
        size_t consumed = 0, samples = 0;
        if (!sbc_decode_frame(msg->payload + off, msg->payload_len - off,
                              &consumed, pcm, &samples)) {

            n_decode_err++;
            sbc_decoder_init();
            break;
        }
        if (consumed == 0) {
            break;
        }
        off += consumed;
        size_t want = samples * sizeof(int16_t);

        if (s_plc_fade_in) {
            const uint32_t frames = (uint32_t)(samples / AUDIO_CHANNELS);
            for (uint32_t i = 0; i < frames && s_plc_fade_in; i++) {
                const int32_t gain =
                    (int32_t)(PLC_FADE_FRAMES - s_plc_fade_in) + 1;
                for (int c = 0; c < AUDIO_CHANNELS; c++) {
                    int16_t *v = &pcm[i * AUDIO_CHANNELS + c];
                    *v = (int16_t)((int32_t)*v * gain / (int32_t)PLC_FADE_FRAMES);
                }
                s_plc_fade_in--;
            }
        }

        plc_note(pcm, (uint32_t)(samples / AUDIO_CHANNELS));

        size_t sent = xStreamBufferSend(ring, pcm, want, 0);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

        visualiser_feed((const uint8_t *)pcm, (uint32_t)sent,
                        msg->play_at + (int64_t)pkt_frames * 1000000 / stream_rate);
        pkt_frames += (uint32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
#endif
        if (sent < want) {
            n_ring_full++;
        }
        samples_in += (int32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
    }
}

static void audio_deliver(const audio_msg_t *msg);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0

_Static_assert(CONFIG_DANCEFLOOR_AUDIO_FEC_K >= 2 &&
               CONFIG_DANCEFLOOR_AUDIO_FEC_K <= AUDIO_FEC_K_MAX,
               "DANCEFLOOR_AUDIO_FEC_K must be 0 (off) or 2..AUDIO_FEC_K_MAX");
#define FEC_K        CONFIG_DANCEFLOOR_AUDIO_FEC_K
#define FEC_HOLD_MAX (FEC_K - 1)

static uint8_t  s_fec_acc[AUDIO_FEC_CODEWORD_MAX];
static uint8_t  s_fec_rec[AUDIO_FEC_CODEWORD_MAX];
static uint16_t s_fec_span;
static uint32_t s_fec_base;
static uint8_t  s_fec_seen;
static bool     s_fec_open;
static bool     s_fec_ok;
static uint32_t s_fec_closed_base;
static bool     s_fec_closed;

static struct {
    bool     active;
    uint32_t seq;
    uint32_t frames;
    uint32_t last_seq;
    int64_t  began_at;
    int      n;
    uint16_t len[FEC_HOLD_MAX];
    uint8_t  buf[FEC_HOLD_MAX][AUDIO_FEC_CODEWORD_MAX];
} s_hold;

static void fec_group_reset(uint32_t base)
{

    memset(s_fec_acc, 0, s_fec_span);
    s_fec_span = 0;
    s_fec_base = base;
    s_fec_seen = 0;
    s_fec_open = true;
    s_fec_ok = true;
}

static void fec_group_close(void)
{
    if (s_fec_open) {
        s_fec_closed_base = s_fec_base;
        s_fec_closed = true;
    }
    s_fec_open = false;
}

static void fec_note_arrival(const audio_msg_t *m)
{
    const uint32_t base = m->seq - (m->seq % FEC_K);

    if (s_fec_closed && base == s_fec_closed_base) {
        return;
    }
    if (!s_fec_open || base != s_fec_base) {
        fec_group_reset(base);
    }
    if (!audio_fec_xor_in(s_fec_acc, &s_fec_span, m)) {
        s_fec_ok = false;
        return;
    }
    s_fec_seen |= (uint8_t)(1u << (m->seq % FEC_K));
}

static bool fec_hold_push(const audio_msg_t *m)
{
    const uint16_t len = (uint16_t)AUDIO_MSG_BYTES(m->payload_len);

    if (s_hold.n >= FEC_HOLD_MAX || len > (uint16_t)AUDIO_FEC_CODEWORD_MAX) {
        return false;
    }
    memcpy(s_hold.buf[s_hold.n], m, len);
    s_hold.len[s_hold.n] = len;
    s_hold.n++;
    s_hold.last_seq = m->seq;
    fec_note_arrival(m);
    return true;
}

static void fec_hold_note_duration(void)
{
    const int64_t held = esp_timer_get_time() - s_hold.began_at;

    if (held > 0 && held < INT32_MAX && (int32_t)held > fec_hold_max_us) {
        fec_hold_max_us = (int32_t)held;
    }
}

static void fec_hold_flush(void)
{
    for (int i = 0; i < s_hold.n; i++) {
        audio_deliver((const audio_msg_t *)s_hold.buf[i]);
    }
    s_hold.n = 0;
}

static void fec_hold_abandon(void)
{
    if (!s_hold.active) {
        return;
    }
    n_fec_lost++;
    fec_hold_note_duration();
    s_hold.active = false;
    fec_group_close();

    if (fill_gap(1, s_hold.frames)) {
        fec_hold_flush();
    } else {

        s_hold.n = 0;
    }
}

static bool fec_stream_ready(void)
{
    return have_seq && stream_start_local != 0;
}

static void fec_reset(void)
{
    if (s_hold.active) {
        n_fec_lost++;
        fec_hold_note_duration();
        s_hold.active = false;
        s_hold.n = 0;
    }
    s_fec_open = false;
    s_fec_closed = false;
    memset(s_fec_acc, 0, s_fec_span);
    s_fec_span = 0;
}

static bool fec_hold_begin(const audio_msg_t *msg, uint32_t missing)
{
    const uint32_t want = expect_seq;

    if (missing != 1) {
        return false;
    }
    if (anchor_provisional) {

        return false;
    }
    if (msg->payload_len > AUDIO_FEC_PAYLOAD_MAX) {
        return false;
    }
    if (want / FEC_K != msg->seq / FEC_K) {
        return false;
    }
    if (!s_fec_open || !s_fec_ok || s_fec_base != want - (want % FEC_K)) {
        return false;
    }

    s_hold.n = 0;
    if (!fec_hold_push(msg)) {
        return false;
    }
    s_hold.active = true;
    s_hold.seq = want;
    s_hold.frames = msg->frames;
    s_hold.began_at = esp_timer_get_time();
    n_fec_holds++;
    return true;
}

static bool fec_hold_offer(const audio_msg_t *m)
{
    if (m->seq != s_hold.last_seq + 1) {
        return false;
    }
    if (m->seq / FEC_K != s_hold.seq / FEC_K) {
        return false;
    }
    if (m->payload_len > AUDIO_FEC_PAYLOAD_MAX) {
        return false;
    }
    return fec_hold_push(m);
}

static bool fec_repair(const audio_fec_msg_t *fm, uint32_t want_seq)
{
    for (uint16_t i = 0; i < fm->span; i++) {
        s_fec_acc[i] ^= fm->parity[i];
    }

    s_fec_span = fm->span;

    if (!audio_fec_extract(s_fec_acc, fm->span, want_seq,
                           (audio_msg_t *)s_fec_rec)) {
        n_fec_bad++;
        return false;
    }

    if (!s_hold.active) {
        n_gaps++;
    }
    n_fec_recovered++;
    if (s_hold.active) {
        fec_hold_note_duration();
        s_hold.active = false;
    }
    fec_group_close();
    audio_deliver((const audio_msg_t *)s_fec_rec);
    fec_hold_flush();
    return true;
}

static void fec_parity_rx(const audio_fec_msg_t *fm, int n)
{
    n_fec_parity_rx++;

    if (!fec_stream_ready()) {
        fec_reset();
        return;
    }

    const bool usable =
        fm->count == FEC_K &&
        fm->span >= (uint16_t)AUDIO_MSG_BYTES(0) &&
        fm->span <= (uint16_t)AUDIO_FEC_CODEWORD_MAX &&
        fm->span >= s_fec_span &&
        n >= (int)AUDIO_FEC_MSG_BYTES(fm->span) &&
        s_fec_open && s_fec_ok && fm->base_seq == s_fec_base;

    if (usable) {
        const uint8_t full = (uint8_t)((1u << FEC_K) - 1u);
        const uint8_t lost = (uint8_t)(full & ~s_fec_seen);

        if (lost == 0) {
            fec_group_close();
            return;
        }

        if ((lost & (uint8_t)(lost - 1u)) == 0) {
            const uint32_t want = s_fec_base + (uint32_t)__builtin_ctz(lost);
            if (want == expect_seq && fec_repair(fm, want)) {
                return;
            }
        }
    } else if (fm->count != FEC_K) {
        n_fec_bad++;
    }

    fec_hold_abandon();
    fec_group_close();
}
#endif

static void audio_deliver(const audio_msg_t *msg)
{
    expect_seq = msg->seq + 1;
    have_seq = true;

    record_packet_positions(msg);
    decode_into_ring(msg);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
    fec_note_arrival(msg);
#endif
}

static void handle_audio(const audio_msg_t *msg, int64_t arrived_at)
{
    int64_t offset;
    bool by_tsf = false;
    if (!clock_offset(&offset, &by_tsf)) {
        return;
    }

    const int64_t lead = msg->play_at - (arrived_at + offset);
    if (lead > LEAD_INSANE_US || lead < -LEAD_INSANE_US) {
        n_lead_insane++;
    } else if (lead < rx_lead_min_us) {
        rx_lead_min_us = (int32_t)lead;
    }
    if (msg->format != AUDIO_FMT_SBC) {
        ESP_LOGW(TAG, "unexpected audio format %u -- hub and satellite disagree",
                 msg->format);
        return;
    }

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0

    if (!fec_stream_ready()) {
        fec_reset();
    }
    if (s_hold.active) {
        if (fec_hold_offer(msg)) {
            return;
        }
        fec_hold_abandon();
    }
#endif

    if ((!have_seq || stream_start_local == 0) &&
        !anchor_stream(msg, offset, by_tsf)) {
        return;
    }
    if (upgrade_provisional_anchor(msg, offset)) {
        return;
    }
    if (!absorb_sequence_gap(msg)) {
        return;
    }

    audio_deliver(msg);
}

void rx_task(void *arg)
{
    (void)arg;

    static uint8_t buf[sizeof(audio_msg_t)];
    _Static_assert(sizeof(buf) >= AUDIO_UDP_MTU, "a full datagram would not fit");

    while (1) {
        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        int64_t t4 = esp_timer_get_time();
        if (n < 1) {

            if (n < 0) {
                n_recv_err++;
                vTaskDelay(1);
            }
            continue;
        }

        if (buf[0] == MSG_TIME_RSP && n >= (int)sizeof(time_msg_t)) {
            time_msg_t msg;
            memcpy(&msg, buf, sizeof(msg));
            sync_est_add(&est, msg.t1, msg.t2, msg.t3, t4);

            est_newest_at = t4;
        } else if (buf[0] == MSG_VOL && n >= (int)sizeof(vol_msg_t)) {
            const vol_msg_t *v = (const vol_msg_t *)buf;

            const uint8_t vol = v->volume > AUDIO_VOL_MAX ? AUDIO_VOL_MAX
                                                          : v->volume;

            n_vol_rx++;

            if (vol != audio_volume || !audio_vol_known) {
                ESP_LOGW(TAG, "VOLUME %u/%d", vol, AUDIO_VOL_MAX);
            }

            audio_volume = vol;
            audio_vol_known = true;
        } else if (buf[0] == MSG_META && n >= (int)sizeof(meta_msg_t)) {
            const link_meta_t *m = (const link_meta_t *)((const meta_msg_t *)buf)->payload;
            ESP_LOGW(TAG, "TRACK #%" PRIu32 ": \"%s\" - %s [%s]",
                     m->track_id, m->title, m->artist, m->album);
        } else if (buf[0] == MSG_FRAME && n >= (int)FRAME_MSG_BYTES(0)) {

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER && CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE

            const frame_msg_t *fm = (const frame_msg_t *)buf;
            if (fm->len == sizeof(vis_frame_t) && fm->count >= 1 &&
                (size_t)fm->count * fm->len <= FRAME_PAYLOAD_MAX &&
                n >= (int)FRAME_MSG_BYTES((size_t)fm->count * fm->len)) {
                for (unsigned i = 0; i < fm->count; i++) {
                    vis_frame_t f;
                    memcpy(&f, fm->payload + i * sizeof(f), sizeof(f));
                    visualiser_submit_frame(&f);
                }

                n_frames_rx += fm->count;
            } else {
                static bool told;
                if (!told) {
                    told = true;
                    ESP_LOGE(TAG, "frame batch of %u x %u bytes in %d -- hub and "
                                  "satellite are not the same build (frame is %u)",
                             fm->count, fm->len, n, (unsigned)sizeof(vis_frame_t));
                }
                n_frames_bad++;
            }
#endif
        } else if (buf[0] == MSG_TSF && n >= (int)sizeof(tsf_msg_t)) {

            tsf_msg_t m;
            memcpy(&m, buf, sizeof(m));

            const int64_t tsf_a = esp_wifi_get_tsf_time(WIFI_IF_STA);
            const int64_t my_local = esp_timer_get_time();
            const int64_t tsf_b = esp_wifi_get_tsf_time(WIFI_IF_STA);
            const int64_t span = tsf_b - tsf_a;
            const int64_t my_tsf = (tsf_a + tsf_b) / 2;

            static bool announced;
            if (!announced) {
                announced = true;
                ESP_LOGW(TAG, "TSF first message: hub %lld us, ours %lld us%s",
                         m.tsf, my_tsf,
                         (m.tsf && my_tsf) ? "" : "  <-- zero means not available");
            }

            int64_t est_offset;
            if (m.tsf == 0 || my_tsf == 0) {
                static bool told_zero;
                if (!told_zero) {
                    told_zero = true;
                    ESP_LOGW(TAG, "TSF unavailable (hub %lld, ours %lld) -- "
                                  "no comparison possible", m.tsf, my_tsf);
                }
                continue;
            }
            if (!sync_est_offset(&est, &est_offset)) {
                continue;
            }

            const int64_t tsf_offset = (m.local - m.tsf) - (my_local - my_tsf);

            if (span > TSF_SPAN_MAX_US) {
                n_tsf_wide++;
            }

            tsf_publish(tsf_offset, my_local);

            static int64_t last_log_us;
            static int64_t prev_tsf, prev_est;
            static int64_t span_max;
            if (span > span_max) {
                span_max = span;
            }
            if (my_local - last_log_us < (int64_t)CONFIG_DANCEFLOOR_LOG_PERIOD_S * 1000000) {
                continue;
            }
            const int64_t tsf_step = last_log_us ? tsf_offset - prev_tsf : 0;
            const int64_t est_step = last_log_us ? est_offset - prev_est : 0;
            last_log_us = my_local;
            prev_tsf = tsf_offset;
            prev_est = est_offset;

            ESP_LOGW(TAG, "TSF: est %+lld us | tsf %+lld us | diff %+lld us | "
                          "steps tsf %+lld us, est %+lld us | span %lld us "
                          "(max %lld)",
                     est_offset, tsf_offset, tsf_offset - est_offset,
                     tsf_step, est_step, span, span_max);
            span_max = 0;
        } else if (buf[0] == MSG_AUDIO && n >= (int)AUDIO_MSG_BYTES(0)) {

            n_audio_rx++;
            if (s_prev_audio_at) {
                const int64_t gap = t4 - s_prev_audio_at;
                if (gap > rx_gap_max_us) {
                    rx_gap_max_us = (int32_t)(gap > INT32_MAX ? INT32_MAX : gap);
                }
                s_burst_run = gap < RX_BURST_US ? s_burst_run + 1 : 1;
            } else {
                s_burst_run = 1;
            }
            if (s_burst_run > rx_burst_max) {
                rx_burst_max = s_burst_run;
            }
            s_prev_audio_at = t4;
            handle_audio((const audio_msg_t *)buf, t4);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
        } else if (buf[0] == MSG_AUDIO_FEC &&
                   n >= (int)AUDIO_FEC_MSG_BYTES(0)) {

            fec_parity_rx((const audio_fec_msg_t *)buf, n);
#endif
        }
    }
}
