/*
 * The timeline this whole system plays to, and the unicast that publishes it.
 *
 * This is the hub's reason for existing. streamer_send_sbc() decides when a
 * packet's audio is due, stamps it, and sends it to every satellite; everything
 * else on the floor -- including this unit's own speaker, via play.c -- is
 * downstream of that number.
 *
 * Also here: streamer_feed(), because the local ring and the phase queue are
 * the two things a packet touches on its way past, and they have to agree about
 * where a packet's audio starts.
 */
#include "hub.h"


/*
 * ONE FUNCTION PER DECISION.
 *
 * streamer_send_sbc was 313 lines carrying seven decisions in a row: whether the
 * source is running, whether to start a timeline or steer the one that exists,
 * what boundary flags this packet carries, where its audio sits in the ring, and
 * who to send it to. The order they are taken in is now the whole of the
 * function; each is a function below, in that order.
 *
 * The bodies are unchanged. The function's own statics moved to file scope with
 * their names intact -- they were already per-call-persistent and are still
 * written by nothing but this file's timeline path -- so no body text was
 * rewritten. The one edit is the steadiness gate's bare `return;` becoming a
 * `return false;` the caller acts on.
 */

/*
 * The timeline itself, and the message being built into.
 *
 * These were function statics of streamer_send_sbc and are file statics now
 * because the decisions that read them are separate functions. Ownership is
 * unchanged: sbc_in's rx_task is the only caller of any of it, so this is
 * single-threaded state despite being at file scope.
 */
static audio_msg_t msg;
static uint32_t seq;
static int64_t next_play_at;
static int64_t s_last_pkt_us;
static int64_t s_steady_since;
static int64_t s_wait_since;

uint32_t streamer_take_dropped(void)
{
    uint32_t d = s_feed_dropped;
    s_feed_dropped = 0;
    return d;
}

void streamer_feed(const uint8_t *pcm, uint32_t len)
{
    /*
     * Straight to the local ring. There is no intermediate PCM buffer any more:
     * satellites receive SBC, so nothing needs PCM except this speaker.
     *
     * Deliberately not gated on local_start. sbc_in decodes and feeds a packet
     * before calling streamer_send_sbc(), which is what sets local_start -- so
     * gating discarded the first packet's audio here while the satellites got
     * it, leaving this unit permanently one packet (~20 ms) behind them. The
     * ring is reset at timeline start, so anything fed early is cleared anyway.
     */
    if (!local_ring) {
        return;
    }
    size_t sent = xStreamBufferSend(local_ring, pcm, len, 0);   /* must not block */
    if (sent < len) {
        s_feed_dropped += len - sent;
    }
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /*
     * The analysis sees the audio here, on arrival, rather than at the DAC.
     *
     * Fed what the ring TOOK, and dated by interpolating from the last packet's
     * (position, play_at) pair, so the count the block grid rides on stays equal
     * to the audio that will actually be played. Before the first packet is sent
     * there is no pair and no timeline, and 0 says so.
     */
    visualiser_feed(pcm, (uint32_t)sent,
                    s_vis_anchor_due
                        ? s_vis_anchor_due + (int64_t)(s_samples_in - s_vis_anchor_pos)
                                             * 1000000LL / (int64_t)sample_rate
                        : 0);
#endif
    s_samples_in += (int32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
}

/* Called as a tagged packet is about to be queued: its audio starts here. */
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

/*
 * Track how steadily the source is delivering, on every packet.
 *
 * Separate from the gate that reads it because it must run whether or not a
 * start is pending -- the answer has to already be there when one is.
 */
static void note_arrival(int64_t now)
{
/*
 * Is the source actually running? See SOURCE_STEADY_US. Tracked on every
 * packet, whether or not a start is pending, so the answer is already there
 * when one is.
 */
if (s_last_pkt_us == 0 || now - s_last_pkt_us > SOURCE_STALL_US) {
    s_steady_since = now;              /* a hole; the run starts again here */
}
s_last_pkt_us = now;
}


/*
 * May a timeline start on what the source is currently doing?
 *
 * False drops this packet, which costs nothing -- see the note below. True means
 * either the source has been clean for SOURCE_STEADY_US or it has stalled so
 * long that refusing to play is the worse failure.
 */
static bool source_steady_enough(int64_t now)
{
    /*
     * Hold the start until the source has gone SOURCE_STEADY_US without a
     * hole. Returning here drops this packet, which costs nothing: the ring
     * is reset at the start anyway, so audio fed before one is discarded
     * regardless, and no satellite can anchor until a timeline exists.
     */
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


/*
 * Publish a new origin. Every unit on the floor anchors to what this sets.
 */
static void start_timeline(int64_t target)
{
    /* Past the gate, so this call will reach the msg.restart assignment and
     * the flag has been delivered. Cleared here rather than where `recovered`
     * was read, so an early return above leaves it set for the next packet. */
    s_underrun_recover = false;
    next_play_at = target;
    /*
     * local_start is assigned at the END of this call, not here.
     *
     * It has to be the stamp of the audio that will actually be at position
     * zero of the ring, and that is NOT this packet: sbc_in decodes and
     * feeds before calling us, so this packet's PCM went into the ring a
     * moment ago and the reset below is about to throw it away. Position
     * zero belongs to the NEXT packet, one packet's worth of audio later.
     *
     * Setting it to `target` here -- which is what this did -- started this
     * speaker on the next packet's audio at the previous packet's instant,
     * so the hub played ~20-45 ms ahead of the timeline it was publishing,
     * on the first start and again after every underrun recovery. The phase
     * servo then read that as a real error and spent ~100 s walking it out,
     * with every satellite that far behind this speaker while it did.
     *
     * The queue entry below already dates position zero correctly, so the
     * two disagreed with each other; this is the half that was wrong.
     */
    xStreamBufferReset(local_ring);
    s_samples_in = 0;                  /* same origin as the reset ring */
    s_pending_pos = 0;                 /* and so does anything flagged here */
    s_vis_anchor_pos = 0;
    s_vis_anchor_due = 0;              /* no timeline until the next packet */
    s_marker_sample = -1;
    s_phase_head = s_phase_tail = 0;
    s_phase_valid = false;
    s_restart_pos = -1;
    s_restart_pending = false;
    s_slewing = false;      /* err is zero by construction; nothing to walk back */
    s_slew_told = false;
    n_restarts++;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /* Frames are drawn when the instant they name comes round, so anything
     * already computed is dated against the origin this line replaces. */
    visualiser_flush();
#endif
    ESP_LOGI(TAG, "timeline start");
}


/*
 * Keep the existing timeline near real time: slew, or jump if it is past saving.
 */
static void steer_timeline(int64_t now, int64_t target)
{
    /*
     * Bring the timeline back to real time by SLEWING, not by jumping.
     *
     * Jumping is what this used to do, and it stepped every unit's phase by
     * the whole error at once -- measured at -126734 us, with both boards
     * then servoing it off over 160 s and drifting 3 to 8 ms apart from
     * each other while they did. That excursion was most of the sync error
     * in a ten-minute run, and the servo was being asked to correct a step
     * with a loop built for 14 ppm of drift.
     *
     * A step is also the wrong description of the fault. Drift accounted
     * for 7% of that -126 ms; the rest arrived as a single ~118 ms pause in
     * delivery, which reduces the playback lead by exactly its own length.
     * Rebuilding that lead is real work and the servo has to do it either
     * way -- but it can do it smoothly if the timeline moves smoothly.
     *
     * The rate is bounded by what the servo can follow. It may trim +-100 Hz
     * at 44.1 kHz, which is 2.27 ms/s, so anything near that leaves the
     * units unable to keep up and puts the error back. 20 us per packet at
     * ~50 packets/s is 1 ms/s, well inside it, and closes 118 ms in about
     * two minutes without the phase ever leaving the servo's linear range.
     */
    const int64_t err = next_play_at - target;

    if (llabs(err) > RESYNC_HARD_US) {
        /* Far beyond anything a slew could close in reasonable time, so
         * something has gone wrong that gradual correction will not fix.
         * Jump, and say so -- this is the old behaviour, kept for the case
         * it was right for. */
        next_play_at = target;
        s_slewing = false;
        /*
         * Tell the satellites, which nothing used to do.
         *
         * A jump of up to RESYNC_HARD_US lands on every unit as a step in the
         * stamps it is receiving, with no splice hint and -- being below
         * PHASE_INSANE_US -- no re-anchor either. Each one then walks it off
         * through its servo at 2.27 ms/s, which for a 300 ms step is over two
         * minutes of every speaker being audibly in a different place.
         *
         * NOT flagged on this packet. A boundary makes each unit splice by
         * its own phase reading, and right now those readings describe the
         * timeline that just stopped existing -- the hub's own included. The
         * flag is armed and fires a few packets later, once the reading it
         * will act on was taken after the discontinuity. SYNC_PHASE_HIST
         * packets is the natural delay: it is how many readings the splice's
         * median window holds, so waiting that long is waiting for the window
         * to be clear of the jump.
         */
        s_jump_arm = SYNC_PHASE_HIST;
        ESP_LOGW(TAG, "timeline off by %lld us -- too far to slew, jumping; "
                      "flagging a boundary in %d packets",
                 err, (int)s_jump_arm);
    } else if (llabs(err) > RESYNC_US) {
        if (!s_slewing) {
            s_slewing = true;
            s_slew_since = now;
        }
        /*
         * Only worth reporting if it PERSISTS. err oscillates with the
         * burst pattern -- the source leaves a ~100 ms gap in every window,
         * so the trough routinely reaches -132 ms against a 120 ms
         * threshold and recovers with the next burst about a second later.
         * Announcing each of those was seven alarms in ninety seconds for
         * something entirely normal.
         */
        if (!s_slew_told && now - s_slew_since > 5000000) {
            s_slew_told = true;
            ESP_LOGW(TAG, "timeline off by %lld us for 5 s, slewing back", err);
        }
    } else if (s_slewing && llabs(err) < RESYNC_US / 4) {
        /* Hysteresis, so it does not chatter in and out around the
         * threshold. */
        s_slewing = false;
        if (s_slew_told) {
            s_slew_told = false;
            ESP_LOGW(TAG, "timeline back within %lld us", err);
        }
    }

    if (s_slewing) {
        next_play_at += (err > 0) ? -TIMELINE_SLEW_US : TIMELINE_SLEW_US;
    }
}


/*
 * What boundary flags this packet carries, and why each one is set.
 */
static void flag_boundaries(bool recovered)
{
/*
 * Two different events wear the same flag on the wire, and they differ in
 * whether this unit splices too.
 *
 * A track boundary: every unit nulls its phase when playback reaches the
 * flagged audio, this one included. That is the case the flag was built for.
 *
 * A timeline restart after a local underrun: the satellites did NOT restart
 * with us. They are still playing against the old timeline, and the stamps
 * they are about to receive step by however far the timeline moved, which is
 * why telling them is worth doing at all. That step is NOT bounded by what a
 * splice can absorb -- this comment used to claim it was capped at RESYNC_US
 * and inside MAX_SPLICE_MS, which stopped being true when RESYNC_US became
 * 150000, exactly the ceiling. A satellite absorbs what it can and servos off
 * the rest. We
 * must not splice: our phase was just re-anchored to zero by construction,
 * while s_phase_err_us still holds whatever it read before the underrun.
 * Acting on that would cut up to MAX_SPLICE_MS out of the first audio of
 * the new timeline for no reason.
 */
/*
 * The armed hard-jump boundary, counted down in packets and fired once. It
 * becomes an ordinary track boundary at that point: every unit splices by its
 * own phase, this one included, and by now every unit's reading was taken
 * after the jump. See where s_jump_arm is set.
 */
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
        s_restart_pos = s_pending_pos;   /* our own copy of the same boundary */
    }
    ESP_LOGW(TAG, "track boundary flagged at seq %" PRIu32, msg.seq);
} else if (recovered) {
    ESP_LOGW(TAG, "timeline restart flagged at seq %" PRIu32
                  " -- satellites re-splice, we do not", msg.seq);
}
}


/*
 * Record where this packet's audio sits and when it is due -- the servo's input,
 * and the analysis's interpolation anchor.
 */
static void record_phase_point(bool started)
{
/*
 * Position captured before the audio was fed, paired with the time it is
 * due -- the playback task compares the two when it gets there.
 *
 * Skipped at a timeline start, and that matters more than it looks.
 * streamer_begin_packet() snapshots s_samples_in before this packet is
 * decoded and fed; the branch above then resets s_samples_in to zero and
 * clears the ring, so the snapshot describes an origin that no longer
 * exists. On the first start it is legitimately zero, but on an underrun
 * restart it is every frame fed since the last one -- hours of them.
 *
 * Queued, that entry sits at the head of a queue playback reaches only
 * after playing the same hours of audio. The tail never advances, the queue
 * fills, no further points are ever recorded, and s_phase_valid stays false
 * for good, so the ring servo stops. The strip used to go with it -- the
 * analysis was dated from the same dead queue, so it was fed a due of zero
 * for ever, hue frozen and no envelope decay, while every satellite carried
 * on. One local underrun took the hub's strip out for the rest of the
 * session. That half is gone now: the analysis is dated from the send side
 * (s_vis_anchor_due), which does not depend on playback reaching anything.
 * The servo half is still worth every line below.
 *
 * Nothing is lost by skipping it. This packet's audio went into the ring
 * before the reset cleared it, so it has no position to record; the next
 * packet lands at zero in the fresh ring and records itself correctly
 * against the timeline this call just advanced.
 */
uint32_t nq = (s_phase_head + 1) % PHASE_Q_LEN;
if (!started && nq != s_phase_tail) {
    s_phase_q[s_phase_head].pos = s_pending_pos;
    s_phase_q[s_phase_head].play_at = next_play_at;
    s_phase_head = nq;
} else if (!started) {
    /* Full. Deliberately not counted at a timeline start, where skipping is
     * correct and routine; here it means playback is not draining the queue
     * and the servo is about to stop getting input. */
    n_phase_drop++;
}
/*
 * The same pair, kept for the analysis, which is fed on arrival and so runs
 * ahead of the instant it is dating audio with.
 *
 * streamer_feed() cannot know this packet's play_at -- sbc_in decodes and
 * feeds before calling here, which is what assigns it. Interpolating from
 * the previous packet's pair covers that: the anchor is one packet stale and
 * the arithmetic is the same interpolation the playback side used to do, so
 * the label is right to the microsecond either way.
 */
s_vis_anchor_pos = s_pending_pos;
s_vis_anchor_due = next_play_at;
}


/*
 * Unicast to every registered listener.
 */
static void fan_out(size_t bytes, int64_t now)
{
/* Age the list before snapshotting it, so the loop below only ever has to
 * skip cleared slots. This used to be interleaved with the send loop and was
 * the ONLY place it happened -- see clients_age(). */
clients_age(now);

client_t snapshot[MAX_CLIENTS];
clients_snapshot(snapshot);

/*
 * Unicast to each registered listener. Multicast was removed entirely: it is
 * never acknowledged and never retried, which cost ~20% of packets at every
 * PHY rate tried (1, 6 and 24 Mbps all landed near the same loss). Unicast
 * gets link-layer ACK and retransmission and measured essentially clean.
 *
 * Airtime now scales with speaker count, which is affordable at ~42 kB/s of
 * SBC and would not have been at 179 kB/s of PCM.
 */
for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!snapshot[i].last_seen) {
        continue;
    }
    if (sendto(sock, &msg, bytes, 0,
               (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {
        s_tx_fail++;
    }
}

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
    /*
     * Recovering from a local underrun. The flag cannot be raised here: the
     * timeline-start branch below clears s_restart_pending, because a start
     * throws away any track boundary that was waiting for the old timeline. So
     * the request to bring the satellites with us was set and then wiped in the
     * same call, and no satellite has ever received it.
     *
     * CONSUMED BELOW, NOT HERE, and that distinction is the whole bug this
     * comment used to describe only half of. Clearing the flag on sight meant the
     * source-steadiness gate could `return` before `msg.restart` was ever written
     * -- and it does return, for up to SOURCE_STEADY_US after a hole, which is
     * exactly the condition an underrun leaves behind. The flag was consumed into
     * a local, thrown away with the packet, and every later packet saw
     * s_underrun_recover already false. So no satellite learned the timeline had
     * restarted, which is the fault the paragraph above was written to fix,
     * reintroduced one line below it.
     *
     * Now it stays set until a packet actually carries it. Re-entering here while
     * still waiting is harmless: next_play_at is already 0 and `recovered` is
     * recomputed from the flag each time.
     */
    bool recovered = false;
    if (s_underrun_recover) {
        next_play_at = 0;              /* fall into the timeline-start path */
        recovered = true;
    }

    /* A timeline start invalidates the position this packet captured, because
     * the ring it referred to is about to be cleared. */
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
    memcpy(msg.payload, sbc, len);

    size_t bytes = AUDIO_MSG_BYTES(len);
    fan_out(bytes, now);

    /* The timeline advances by the audio actually sent, not by wall clock --
     * stamping "now + lead" each time would fold task jitter into playback. */
    next_play_at += (int64_t)frames * 1000000LL / (int64_t)sample_rate;

    /*
     * Our own speaker joins the timeline, on the audio that will be at position
     * zero of the freshly reset ring -- the next packet, whose stamp is the one
     * this call has just left behind. Written here rather than in the branch
     * above so that it is the same expression as the timeline itself, and cannot
     * drift from it again.
     */
    if (started) {
        local_start = next_play_at;
        /* Published AFTER the instant, and it is what the play task waits on.
         * Writing the value first is the whole of the ordering guarantee -- see
         * the declaration in hub.h. */
        local_epoch++;
    }
}

