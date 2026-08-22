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
/*
 * The sub-microsecond remainder of the advance below, in 1/sample_rate of a
 * microsecond. See where it is spent for why a timeline that throws it away
 * loses 93 ms an hour.
 */
static int64_t s_play_at_rem;
static int64_t s_last_pkt_us;
static int64_t s_steady_since;
static int64_t s_wait_since;
/* When the starving-ring hold in steer_timeline() began; 0 = not holding.
 * See TIMELINE_HOLD_STARVE_MS. */
static int64_t s_hold_since;

#if CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH > 0
/*
 * The last FEC_DEPTH payloads, kept so each outgoing packet can piggyback them
 * as trailing redundancy for the satellite to recover a loss from. [0] is the
 * packet immediately before this one, [1] the one before that. Single-writer
 * like everything else in this file: sbc_in's rx_task is the only caller.
 */
static struct {
    uint8_t  payload[AUDIO_MAX_PAYLOAD];
    uint16_t len;
    uint32_t seq;
    bool     valid;
} fec_prev[CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH];
#endif

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
    /* The carry described the timeline this call replaces. Same reason
     * s_trim_owed is reset at a playback start in both play.c files: a fraction
     * banked against a position nothing occupies any more is not owed. */
    s_play_at_rem = 0;
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
        /*
         * HELD, NOT JUMPED, while the local ring is starving, since
         * 2026-08-18. See TIMELINE_HOLD_STARVE_MS for the full argument:
         * a starved ring manufactures this error all by itself, because
         * next_play_at stops advancing while target keeps moving with
         * `now`. Jumping re-stamps the fleet to fix delivery, and each
         * jump arms a boundary every unit splices at -- on the 2026-08-18
         * soak, five jumps in 45 s during a minute of source rate wander,
         * four audible splices per unit, the rings then ballooned past
         * 430 ms by the inserts. Held, the same excursion recovered on
         * its own the moment the burst arrived.
         *
         * Held means held: no jump, no s_jump_arm, nothing but the one log
         * line. If the error is still past saving when the ring refills,
         * the jump below takes it on the next packet; if the burst has
         * unwound it, nothing was re-stamped and no unit ever spliced.
         * s_slewing is left exactly as it was, and the trailing nudge
         * still runs: at 20 us/packet a residual slew during the hold is
         * a rounding error either way.
         *
         * The give-up bounds a hold that never ends. The cleaner escape
         * usually arrives first: a ring starved all the way to empty ends
         * in CHUNK_UNDERRUN -> s_underrun_recover -> start_timeline(), a
         * wholesale restart every satellite re-anchors on, rather than a
         * step in the one that exists.
         */
        const int32_t level_ms = (int32_t)((int64_t)(LOCAL_RING_BYTES -
                xStreamBufferSpacesAvailable(local_ring)) * 1000
                / ((int64_t)sample_rate * AUDIO_CHANNELS * 2));
        const bool starving = level_ms < TIMELINE_HOLD_STARVE_MS;
        bool held = false;
        if (starving && (!s_hold_since ||
                         now - s_hold_since < TIMELINE_HOLD_GIVE_UP_US)) {
            /* ONE line per episode, not one per packet: this branch is
             * reached on every send for as long as the starvation lasts. */
            if (!s_hold_since) {
                s_hold_since = now;
                ESP_LOGW(TAG, "timeline off by %lld us but local ring at "
                              "%ld ms -- holding the jump; the source is "
                              "starving, not the clock",
                         err, (long)level_ms);
            }
            held = true;
        } else {
            s_hold_since = 0;   /* ring recovered, or the hold outlived its bound */
        }
        if (!held) {
            /* Far beyond anything a slew could close in reasonable time, so
             * something has gone wrong that gradual correction will not fix.
             * Jump, and say so -- this is the old behaviour, kept for the case
             * it was right for. */
            next_play_at = target;
            s_play_at_rem = 0;      /* same reason as in start_timeline() */
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
        }
    } else {
        s_hold_since = 0;   /* the excursion is over; re-arm the one-shot log */
        if (llabs(err) > RESYNC_US) {
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
 * The audio downlink: one sendto per registered satellite. Everything else
 * about a packet (its format, the FEC redundancy attached in
 * streamer_send_sbc(), the socket it leaves on, the satellite's receive path)
 * is decided elsewhere, so all that happens here is the walk over the list.
 */
/*
 * What became of one packet's sends, so fan_out() can time the gap between
 * packets that actually REACHED THE AIR.
 *
 * The distinction is not academic and this counter was wrong without it. On the
 * 2026-08-19 19:36 soak the hub refused 48 audio sends inside one minute --
 * `tx-fail 51 (48 audio) -- Not enough space` -- while the satellite recorded
 * arrival gaps of 742, 472 and 505 ms and starved its DAC for 1.75 s. The gauge
 * read a perfectly normal 58 ms throughout, because fan_out() was CALLED on
 * schedule every 20 ms; it was the sendto inside it that failed. An instrument
 * built to say whether the hub emitted evenly answered "yes" for the one event
 * where it emphatically had not.
 *
 * NO_LISTENERS is its own answer rather than folded into REFUSED, because the
 * two mean opposite things about the link: nothing was refused, there was simply
 * nobody to send to, and an empty floor must not accumulate a gap that then
 * reads as a spike the moment a satellite joins.
 */
typedef enum {
    FANOUT_SENT,          /* at least one sendto took it */
    FANOUT_REFUSED,       /* every sendto was refused -- a real hole */
    FANOUT_NO_LISTENERS,  /* nothing to send to, so nothing to measure */
} fanout_result_t;

/*
 * Unicast to each registered listener. Multicast was removed entirely: it is
 * never acknowledged and never retried, which cost ~20% of packets at every
 * PHY rate tried (1, 6 and 24 Mbps all landed near the same loss). Unicast
 * gets link-layer ACK and retransmission and measured essentially clean.
 *
 * Airtime now scales with speaker count, which is affordable at ~42 kB/s of
 * SBC and would not have been at 179 kB/s of PCM.
 */
static fanout_result_t send_audio_to_clients(size_t bytes)
{
    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    /* One satellite taking it is the packet reaching the air: the gauge asks
     * whether the hub emitted, not whether every listener was served, and a
     * per-listener failure is already counted by tx_fail_note_audio. */
    bool any_listener = false, any_sent = false;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!snapshot[i].last_seen) {
            continue;
        }
        any_listener = true;
        if (sendto(sock, &msg, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {
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

/*
 * Send the audio packet built in streamer_send_sbc() to whoever is listening.
 * Age the client list first so the snapshot below only ever skips a cleared
 * slot, never acts on a stale one; telemetry.c ages it too, so the second age
 * is redundant but cheap.
 */
static void fan_out(size_t bytes, int64_t now)
{
    /* Counted here rather than at the sendto, so it is the rate the
     * timeline slews at and the rate the TX pool sees, not the rate that
     * happened to succeed. See s_audio_pkts. */
    s_audio_pkts++;
    clients_age(now);
    const fanout_result_t r = send_audio_to_clients(bytes);

    /*
     * How far apart two audio packets actually REACHED THE AIR -- the hub's
     * half of the satellite's arrival gauges (sat.h, rx_gap_max_us).
     *
     * The satellite can see a 200 ms hole in its arrivals and cannot tell
     * whether the packets were sent late or delayed on the way. This is the
     * other end of that question, and it has to be measured on this side
     * because this is the only place that knows when the hub let go of one.
     * Steady state is the ~20 ms of one SBC packet, with the source's own
     * lumpiness on top: the 2026-08-19 soak read 54-77 ms per window against a
     * satellite seeing 39-95, which is the two ends agreeing that the air adds
     * nothing.
     *
     * AFTER THE SEND, AND ONLY ON SUCCESS. It used to sit before it, timing
     * calls rather than transmissions, and fanout_result_t's comment has the
     * soak where that made the gauge say "even" through the worst minute in the
     * capture. A refused packet now leaves the previous stamp standing, so the
     * next success measures the WHOLE hole -- which is exactly the reading the
     * satellite saw and this end did not.
     *
     * NO_LISTENERS resets the stamp instead: there is no hole when there is
     * nobody to send to, and carrying one across an empty floor would report a
     * spike at the moment a satellite joined.
     *
     * A gauge, cleared by the window that prints it.
     */
    static int64_t s_fanout_prev_at;
    if (r == FANOUT_NO_LISTENERS) {
        s_fanout_prev_at = 0;
        return;
    }
    if (r != FANOUT_SENT) {
        return;                     /* refused: the hole is still open */
    }
    /* The pool handed a buffer back. This is what closes a refusal stretch for
     * tx_burst_summary(), and it has to be the audio lane that does it -- see
     * the note above enomem_note_shape() in net.c. */
    tx_send_ok();
    if (s_fanout_prev_at) {
        const int64_t gap = now - s_fanout_prev_at;
        if (gap > n_fanout_gap_max_us) {
            n_fanout_gap_max_us = (int32_t)(gap > INT32_MAX ? INT32_MAX : gap);
        }
    }
    s_fanout_prev_at = now;
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
    /*
     * How much lead this packet is going out with -- see n_lead_min_us in hub.h
     * for what it is paired against and why it was the missing half.
     *
     * Here, at the stamp, and against the `now` read at the top of this call:
     * that is the same instant the satellite's arrival lead is measured from at
     * the other end, so the two subtract to a transit time. Everything between
     * this line and the sendto is microseconds of FEC assembly, so it does not
     * matter which side of it this sits.
     */
    {
        const int64_t lead = next_play_at - now;
        /* Refused rather than clamped, exactly as the satellite's copy is: a
         * gauge that clamps reports its own rail as a measurement, which cost
         * a whole run's summary on the 2026-08-19 23:19 soak. This end cannot
         * see the offset step that did it there -- both terms are this board's
         * own clock -- but the two ends must not disagree about what a lead is,
         * and a bound that never fires is the cheap half of that. */
        if (lead <= LEAD_INSANE_US && lead >= -LEAD_INSANE_US &&
            lead < n_lead_min_us) {
            n_lead_min_us = (int32_t)lead;
        }
    }
    memcpy(msg.payload, sbc, len);

    size_t bytes = AUDIO_MSG_BYTES(len);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH > 0
    /*
     * Piggyback up to FEC_DEPTH previous payloads after this one, as far as the
     * UDP MTU allows. Each block is an audio_red_hdr_t plus its SBC; red_seq_ofs
     * is how many packets back it recovers (1, 2, ...). Attached only while the
     * stored chain is contiguous -- if a packet was skipped here (an early
     * return above left a hole in seq), the deeper slots no longer describe a
     * run and are left off. The blocks are written into the unused tail of
     * msg.payload[], which is AUDIO_MAX_PAYLOAD bytes and so always holds a
     * packet that itself fits the MTU.
     *
     * COPIES ARE TRUNCATED, and n_fec_truncated counts it. An ~825-byte payload
     * leaves ~618 bytes of room, so about a quarter of each copy is missing and
     * the satellite pads that much silence -- which is why the depth defaults to
     * 0 and this loop does not run. Sizing the payload so a copy fits whole was
     * tried and reverted; DANCEFLOOR_AUDIO_FEC_DEPTH's Kconfig help has the
     * measurement.
     */
    {
        size_t off = AUDIO_MSG_BYTES(len);          /* where the next block goes */
        for (int d = 0; d < CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH; d++) {
            if (!fec_prev[d].valid) {
                break;
            }
            if (fec_prev[d].seq + (uint32_t)(d + 1) != msg.seq) {
                break;                              /* chain broken by a skip */
            }
            if (off + AUDIO_RED_HDR_BYTES > AUDIO_UDP_MTU) {
                break;
            }
            size_t room = AUDIO_UDP_MTU - off - AUDIO_RED_HDR_BYTES;
            uint16_t take = fec_prev[d].len;
            if (take > room) {
                take = (uint16_t)room;
                n_fec_truncated++;
            }
            audio_red_hdr_t rh = { .red_len = take, .red_seq_ofs = (uint8_t)(d + 1) };
            uint8_t *dst = (uint8_t *)&msg + off;
            memcpy(dst, &rh, AUDIO_RED_HDR_BYTES);
            memcpy(dst + AUDIO_RED_HDR_BYTES, fec_prev[d].payload, take);
            off += AUDIO_RED_HDR_BYTES + take;
        }
        bytes = off;
    }
#endif
    fan_out(bytes, now);

#if CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH > 0
    /* Shift the ring: this packet becomes [0], older ones move down. After the
     * send so an early return (which never reached here) does not poison the
     * chain with a packet that was not transmitted. */
    for (int d = CONFIG_DANCEFLOOR_AUDIO_FEC_DEPTH - 1; d > 0; d--) {
        fec_prev[d] = fec_prev[d - 1];
    }
    memcpy(fec_prev[0].payload, sbc, len);
    fec_prev[0].len = len;
    fec_prev[0].seq = msg.seq;
    fec_prev[0].valid = true;
#endif

    /*
     * The timeline advances by the audio actually sent, not by wall clock --
     * stamping "now + lead" each time would fold task jitter into playback.
     *
     * THE REMAINDER IS CARRIED, and throwing it away was the fault behind the
     * whole "the floor loses sync every few minutes" complaint. This line runs
     * ~50 times a second forever, and `frames` is a whole number of decoded SBC
     * frames -- a multiple of the codec's 16/32/48/64/96/128 samples -- so the
     * division is exact only when `frames` is a multiple of 441, which no SBC
     * frame count ever is. 896 frames is 20317.4603 us and used to store 20317;
     * 768 is 17414.9660 and stored 17414. Half a microsecond a packet, 26 us a
     * second, 26 ppm, 93 ms an hour.
     *
     * What that bought, on the 2026-08-19 18:36 soak: the lead in front of every
     * stamp shrank at 122 ms/hour (measured -122.2 on the hub's own ring and
     * -121.5 on the satellite's ring-low, two independent estimators agreeing to
     * three digits). Nothing corrects it -- steer_timeline() above only slews
     * past RESYNC_US, 150 ms, and the servo's depth net only clamps past 120 ms,
     * so the drift lives entirely inside both deadbands. After 45 minutes the
     * satellite's margin was 87 ms, an ordinary 93 ms delivery gap emptied its
     * ring, the DAC played auto_clear silence, and the catch-up drain corrected
     * the resulting lateness at +3.1% for nine seconds. That is what the room
     * heard, and it began here.
     *
     * Both symptoms have this sign: a next_play_at that advances slower than the
     * audio it describes carries less lead than LEAD_US to the satellites, and
     * starts this unit's own playback sooner relative to the audio fed
     * (local_start below), so the local ring drains too.
     *
     * The accumulator is the same shape as the fine trim's TRIM_ONE_FRAME in
     * both play.c files, and for the same reason: a correction far below the
     * unit of the thing being corrected has to be banked until it is worth one.
     * Exact over any number of packets -- the loss becomes zero, not smaller.
     */
    const int64_t advance = (int64_t)frames * 1000000LL + s_play_at_rem;
    next_play_at += advance / (int64_t)sample_rate;
    s_play_at_rem = advance % (int64_t)sample_rate;

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

