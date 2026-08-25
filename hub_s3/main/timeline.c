
#include "hub.h"

static audio_msg_t msg;
static uint32_t seq;
static int64_t next_play_at;

static int64_t s_play_at_rem;
static int64_t s_last_pkt_us;
static int64_t s_steady_since;
static int64_t s_wait_since;

static int64_t s_hold_since;

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
_Static_assert(CONFIG_DANCEFLOOR_AUDIO_FEC_K >= 2 &&
               CONFIG_DANCEFLOOR_AUDIO_FEC_K <= AUDIO_FEC_K_MAX,
               "DANCEFLOOR_AUDIO_FEC_K must be 0 (off) or 2..AUDIO_FEC_K_MAX");
#define FEC_K CONFIG_DANCEFLOOR_AUDIO_FEC_K

static audio_fec_msg_t *s_fec;
static uint16_t s_fec_span;
static uint32_t s_fec_base;

static uint32_t s_fec_have;
static bool     s_fec_ok;
#endif

uint32_t streamer_take_dropped(void)
{
    uint32_t d = s_feed_dropped;
    s_feed_dropped = 0;
    return d;
}

void streamer_feed(const uint8_t *pcm, uint32_t len)
{

    if (!local_ring) {
        return;
    }
    size_t sent = xStreamBufferSend(local_ring, pcm, len, 0);
    if (sent < len) {
        s_feed_dropped += len - sent;
    }
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

    visualiser_feed(pcm, (uint32_t)sent,
                    s_vis_anchor_due
                        ? s_vis_anchor_due + (int64_t)(s_samples_in - s_vis_anchor_pos)
                                             * 1000000LL / (int64_t)sample_rate
                        : 0);
#endif
    s_samples_in += (int32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
}

void streamer_mark_here(void)
{
    if (s_marker_sample < 0) {
        s_marker_sample = s_samples_in;
    }
}

void streamer_request_restart(void)
{
    s_restart_pending = true;
}

void streamer_begin_packet(void)
{
    s_pending_pos = s_samples_in;
}

static void note_arrival(int64_t now)
{

    if (s_last_pkt_us == 0 || now - s_last_pkt_us > SOURCE_STALL_US) {
        s_steady_since = now;
    }
    s_last_pkt_us = now;
}

static bool source_steady_enough(int64_t now)
{

    if (now - s_steady_since < SOURCE_STEADY_US) {
        if (s_wait_since == 0) {
            s_wait_since = now;
        }
        if (now - s_wait_since < SOURCE_GIVE_UP_US) {
            return false;
        }
        ESP_LOGW(TAG, "source still stalling after %lld ms -- starting the "
                      "timeline on it anyway",
                 (now - s_wait_since) / 1000);
    }
    s_wait_since = 0;
    return true;
}

static void start_timeline(int64_t target)
{

    s_underrun_recover = false;
    next_play_at = target;

    s_play_at_rem = 0;

    xStreamBufferReset(local_ring);
    s_samples_in = 0;
    s_pending_pos = 0;
    s_vis_anchor_pos = 0;
    s_vis_anchor_due = 0;
    s_marker_sample = -1;
    s_phase_head = s_phase_tail = 0;
    s_phase_valid = false;
    s_restart_pos = -1;
    s_restart_pending = false;
    s_slewing = false;
    s_slew_told = false;
    n_restarts++;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

    visualiser_flush();
#endif
    ESP_LOGI(TAG, "timeline start");
}

static void steer_timeline(int64_t now, int64_t target)
{

    const int64_t err = next_play_at - target;

    if (llabs(err) > RESYNC_HARD_US) {

        const int32_t level_ms = (int32_t)((int64_t)(LOCAL_RING_BYTES -
                xStreamBufferSpacesAvailable(local_ring)) * 1000
                / ((int64_t)sample_rate * AUDIO_CHANNELS * 2));
        const bool starving = level_ms < TIMELINE_HOLD_STARVE_MS;
        bool held = false;
        if (starving && (!s_hold_since ||
                         now - s_hold_since < TIMELINE_HOLD_GIVE_UP_US)) {

            if (!s_hold_since) {
                s_hold_since = now;
                ESP_LOGW(TAG, "timeline off by %lld us but local ring at "
                              "%ld ms -- holding the jump; the source is "
                              "starving, not the clock",
                         err, (long)level_ms);
            }
            held = true;
        } else {
            s_hold_since = 0;
        }
        if (!held) {

            next_play_at = target;
            s_play_at_rem = 0;
            s_slewing = false;

            s_jump_arm = SYNC_PHASE_HIST;
            ESP_LOGW(TAG, "timeline off by %lld us -- too far to slew, jumping; "
                          "flagging a boundary in %d packets",
                     err, (int)s_jump_arm);
        }
    } else {
        s_hold_since = 0;
        if (llabs(err) > RESYNC_US) {
            if (!s_slewing) {
                s_slewing = true;
                s_slew_since = now;
            }

            if (!s_slew_told && now - s_slew_since > 5000000) {
                s_slew_told = true;
                ESP_LOGW(TAG, "timeline off by %lld us for 5 s, slewing back", err);
            }
        } else if (s_slewing && llabs(err) < RESYNC_US / 4) {

            s_slewing = false;
            if (s_slew_told) {
                s_slew_told = false;
                ESP_LOGW(TAG, "timeline back within %lld us", err);
            }
        }
    }

    if (s_slewing) {
        next_play_at += (err > 0) ? -TIMELINE_SLEW_US : TIMELINE_SLEW_US;
    }
}

static void flag_boundaries(bool recovered)
{

    if (s_jump_arm && --s_jump_arm == 0) {
        s_restart_pending = true;
        if (s_restart_pos < 0) {
            s_restart_pos = s_pending_pos;
        }
        ESP_LOGW(TAG, "post-jump boundary now flagged -- phase readings are clear "
                      "of the discontinuity");
    }

    msg.restart = (s_restart_pending || recovered) ? 1 : 0;
    if (s_restart_pending) {
        s_restart_pending = false;
        if (s_restart_pos < 0) {
            s_restart_pos = s_pending_pos;
        }
        ESP_LOGW(TAG, "track boundary flagged at seq %" PRIu32, msg.seq);
    } else if (recovered) {
        ESP_LOGW(TAG, "timeline restart flagged at seq %" PRIu32
                      " -- satellites re-splice, we do not", msg.seq);
    }
}

static void record_phase_point(bool started)
{

    uint32_t nq = (s_phase_head + 1) % PHASE_Q_LEN;
    if (!started && nq != s_phase_tail) {
        s_phase_q[s_phase_head].pos = s_pending_pos;
        s_phase_q[s_phase_head].play_at = next_play_at;
        s_phase_head = nq;
    } else if (!started) {

        n_phase_drop++;
    }

    s_vis_anchor_pos = s_pending_pos;
    s_vis_anchor_due = next_play_at;
}

typedef enum {
    FANOUT_SENT,
    FANOUT_REFUSED,
    FANOUT_NO_LISTENERS,
} fanout_result_t;

static fanout_result_t send_audio_to_clients(size_t bytes)
{
    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    bool any_listener = false, any_sent = false;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!snapshot[i].last_seen) {
            continue;
        }
        any_listener = true;
        if (sendto(sock, &msg, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {

            if (errno == ENOMEM) {
                n_audio_retry++;
                if (sendto(sock, &msg, bytes, 0,
                           (struct sockaddr *)&snapshot[i].addr,
                           sizeof(snapshot[i].addr)) >= 0) {
                    n_audio_retry_ok++;
                    any_sent = true;
                    continue;
                }
            }
            tx_fail_note_audio(errno);
        } else {
            any_sent = true;
        }
    }
    if (!any_listener) {
        return FANOUT_NO_LISTENERS;
    }
    return any_sent ? FANOUT_SENT : FANOUT_REFUSED;
}

static void fan_out(size_t bytes, int64_t now)
{

    s_audio_pkts++;
    clients_age(now);
    const fanout_result_t r = send_audio_to_clients(bytes);

    static int64_t s_fanout_prev_at;
    if (r == FANOUT_NO_LISTENERS) {
        s_fanout_prev_at = 0;
        return;
    }
    if (r != FANOUT_SENT) {
        return;
    }

    tx_send_ok();
    if (s_fanout_prev_at) {
        const int64_t gap = now - s_fanout_prev_at;
        if (gap > n_fanout_gap_max_us) {
            n_fanout_gap_max_us = (int32_t)(gap > INT32_MAX ? INT32_MAX : gap);
        }
    }
    s_fanout_prev_at = now;
}

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0

static void send_fec_to_clients(size_t bytes)
{
    if (esp_timer_get_time() < s_tx_congested_until) {
        n_fec_cong_skip++;
        return;
    }

    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    bool any = false;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!snapshot[i].last_seen) {
            continue;
        }

        if (sendto(sock, s_fec, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr,
                   sizeof(snapshot[i].addr)) < 0) {
            tx_fail_note(TX_LANE_FEC, errno);
        } else {
            any = true;
        }
    }
    if (any) {
        n_fec_sent++;
    }
}

static void fec_note_sent(const audio_msg_t *m, int64_t now)
{
    (void)now;
    if (!s_fec) {
        return;
    }
    const uint32_t idx = m->seq % FEC_K;

    if (idx == 0) {

        memset(s_fec->parity, 0, s_fec_span);
        s_fec_span = 0;
        s_fec_base = m->seq;
        s_fec_have = 0;
        s_fec_ok = true;
    }

    if (s_fec_ok && !audio_fec_xor_in(s_fec->parity, &s_fec_span, m)) {
        s_fec_ok = false;
    }
    s_fec_have = idx + 1;

    if (idx + 1 < FEC_K) {
        return;
    }
    if (!s_fec_ok) {
        n_fec_skipped++;
        return;
    }

    s_fec->type = MSG_AUDIO_FEC;
    s_fec->count = FEC_K;
    s_fec->span = s_fec_span;
    s_fec->base_seq = s_fec_base;
    send_fec_to_clients(AUDIO_FEC_MSG_BYTES(s_fec_span));
}
#endif

void streamer_fec_start(void)
{
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
    s_fec = heap_caps_malloc(sizeof(*s_fec), MALLOC_CAP_SPIRAM);
    if (!s_fec) {
        ESP_LOGW(TAG, "XOR parity disabled: PSRAM refused %u bytes. Audio is "
                      "unaffected; a lost packet is a gap again, as it was "
                      "before DANCEFLOOR_AUDIO_FEC_K existed.",
                 (unsigned)sizeof(*s_fec));
        return;
    }
    memset(s_fec, 0, sizeof(*s_fec));
    ESP_LOGI(TAG, "XOR parity: K=%d, one %u-byte parity per group, %u%% overhead",
             FEC_K, (unsigned)sizeof(*s_fec), 100 / FEC_K);
#endif
}

void streamer_send_sbc(const uint8_t *sbc, uint16_t len, uint32_t frames, bool marker)
{
    if (len > AUDIO_MAX_PAYLOAD) {
        n_wifi_oversize++;
        return;
    }
    if (sock < 0 || len == 0 || frames == 0) {
        return;
    }

    int64_t now = esp_timer_get_time();
    int64_t target = now + LEAD_US;
    note_arrival(now);

    bool recovered = false;
    if (s_underrun_recover) {
        next_play_at = 0;
        recovered = true;
    }

    bool started = false;
    if (next_play_at == 0) {
        if (!source_steady_enough(now)) {
            return;
        }
        start_timeline(target);
        started = true;
    } else {
        steer_timeline(now, target);
    }

    msg.type = MSG_AUDIO;
    msg.format = AUDIO_FMT_SBC;
    msg.payload_len = len;
    msg.seq = seq++;
    msg.sample_rate = sample_rate;
    msg.frames = frames;
    msg.marker = marker ? 1 : 0;
    flag_boundaries(recovered);
    record_phase_point(started);

    msg.play_at = next_play_at;

    {
        const int64_t lead = next_play_at - now;

        if (lead <= LEAD_INSANE_US && lead >= -LEAD_INSANE_US &&
            lead < n_lead_min_us) {
            n_lead_min_us = (int32_t)lead;
        }
    }
    memcpy(msg.payload, sbc, len);

    const size_t bytes = AUDIO_MSG_BYTES(len);
    fan_out(bytes, now);

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0

    fec_note_sent(&msg, now);
#endif

    const int64_t advance = (int64_t)frames * 1000000LL + s_play_at_rem;
    next_play_at += advance / (int64_t)sample_rate;
    s_play_at_rem = advance % (int64_t)sample_rate;

    if (started) {
        local_start = next_play_at;

        local_epoch++;
    }
}
