
/**
 * @file timeline.c
 * @brief The timeline this whole system plays to, and the unicast that
 *        publishes it.
 *
 * This is the hub's reason for existing. streamer_send_sbc() decides when a
 * packet's audio is due, stamps it, and sends it to every satellite;
 * everything else on the floor -- including this unit's own speaker, via
 * play.c -- is downstream of that number. One function per decision, in the
 * order streamer_send_sbc() takes them: whether the source is running, whether
 * to start a timeline or steer the one that exists, what boundary flags this
 * packet carries, where its audio sits in the ring, and who to send it to.
 *
 * streamer_feed() is here too, because the local ring and the phase queue are
 * the two things a packet touches on its way past and they have to agree about
 * where its audio starts.
 *
 * Everything at file scope below is single-threaded state despite being at
 * file scope: sbc_in's rx_task is the only caller of any of it.
 */
#include "hub.h"

/** @brief The audio datagram being built. Reused rather than rebuilt: only the
 *         payload and a handful of header fields change per packet. */
static audio_msg_t msg;
/** @brief Packet counter, assigned as `seq++` on the single path that reaches
 *         fan_out() and never reset. The FEC grouping below is derived from
 *         it, so it must stay monotonic. */
static uint32_t seq;
/** @brief When the next packet's audio is due, in master (= local) time. This
 *         is the timeline. */
static int64_t next_play_at;
/** @brief Sub-microsecond remainder of the timeline advance, in 1/sample_rate
 *         of a microsecond. See where it is spent for why a timeline that
 *         throws it away loses ~93 ms an hour. */
static int64_t s_play_at_rem;
/** @brief When the previous packet arrived, for the stall test. */
static int64_t s_last_pkt_us;
/** @brief Start of the current hole-free run of arrivals. See
 *         SOURCE_STEADY_US. */
static int64_t s_steady_since;
/** @brief When the start began waiting on that run; 0 = not waiting. Bounded
 *         by SOURCE_GIVE_UP_US. */
static int64_t s_wait_since;
/** @brief When the starving-ring hold in steer_timeline() began; 0 = not
 *         holding. See TIMELINE_HOLD_STARVE_MS. */
static int64_t s_hold_since;

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
_Static_assert(CONFIG_DANCEFLOOR_AUDIO_FEC_K >= 2 &&
               CONFIG_DANCEFLOOR_AUDIO_FEC_K <= AUDIO_FEC_K_MAX,
               "DANCEFLOOR_AUDIO_FEC_K must be 0 (off) or 2..AUDIO_FEC_K_MAX");
/** @brief Packets per parity group. */
#define FEC_K CONFIG_DANCEFLOOR_AUDIO_FEC_K

/**
 * @brief The parity datagram for the group being sent, accumulated IN PLACE
 *        and held IN PSRAM.
 *
 * One buffer, not two: audio_fec_xor_in() folds each packet straight into the
 * message that will carry it, so the group's running XOR and the datagram are
 * the same bytes rather than an accumulator plus a copy of it.
 *
 * And not in internal DRAM, which is the constraint the whole scheme had to be
 * designed around. The WiFi driver's static TX buffers are DMA-capable
 * internal SRAM and cannot live anywhere else, so internal DRAM here runs
 * tight; spending a datagram of it on redundancy would be taking it from the
 * pool whose exhaustion is what drops the audio in the first place. PSRAM has
 * megabytes and this buffer never touches DMA -- sendto() copies it into a
 * pbuf -- so the slow read is one datagram per group and nothing measures it.
 *
 * Allocated once, in streamer_fec_start(), and parity is simply off if the
 * board has no PSRAM to give.
 */
static audio_fec_msg_t *s_fec;
/** @brief Bytes of s_fec->parity[] in use this group. */
static uint16_t s_fec_span;
/** @brief seq of member 0 of the group. */
static uint32_t s_fec_base;
/**
 * @brief Members folded in so far, which is also the assertion that they are
 *        the contiguous run the parity will claim they are.
 *
 * msg.seq is assigned as `seq++` on the single path that reaches fan_out() and
 * is never reset, so this always equals the arriving member's index and a
 * group is always a whole run. Nothing enforces that here: it is a property of
 * the code above, and a check for a state that cannot occur is worth less than
 * the sentence saying why.
 */
static uint32_t s_fec_have;
/** @brief false once this group cannot produce parity. */
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
    /*
     * Straight to the local ring. There is no intermediate PCM buffer:
     * satellites receive SBC, so nothing needs PCM except this speaker.
     *
     * Deliberately not gated on local_start. sbc_in decodes and feeds a packet
     * before calling streamer_send_sbc(), which is what sets local_start -- so
     * gating would discard the first packet's audio here while the satellites
     * got it, leaving this unit permanently one packet behind them. The ring
     * is reset at timeline start, so anything fed early is cleared anyway.
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
     * Fed what the ring TOOK, and dated by interpolating from the last
     * packet's (position, play_at) pair, so the count the block grid rides on
     * stays equal to the audio that will actually be played. Before the first
     * packet is sent there is no pair and no timeline, and 0 says so.
     */
    visualiser_feed(pcm, (uint32_t)sent,
                    s_vis_anchor_due
                        ? s_vis_anchor_due + (int64_t)(s_samples_in - s_vis_anchor_pos)
                                             * 1000000LL / (int64_t)sample_rate
                        : 0);
#endif
    s_samples_in += (int32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
}

/* Called as a tagged packet is about to be queued: its audio starts here.
 * Declared in streamer.h, like the four entry points around it. */
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

/**
 * @brief Track how steadily the source is delivering.
 *
 * Separate from the gate that reads it because it must run on every packet
 * whether or not a start is pending -- the answer has to already be there when
 * one is. See SOURCE_STEADY_US.
 *
 * @param now  Arrival instant of this packet, us.
 */
static void note_arrival(int64_t now)
{
    if (s_last_pkt_us == 0 || now - s_last_pkt_us > SOURCE_STALL_US) {
        s_steady_since = now;   /* a hole; the run starts again here */
    }
    s_last_pkt_us = now;
}

/**
 * @brief May a timeline start on what the source is currently doing?
 *
 * @param now  Arrival instant of this packet, us.
 * @return false to drop this packet, which costs nothing: the ring is reset at
 *         a start anyway, so audio fed before one is discarded regardless, and
 *         no satellite can anchor until a timeline exists. true means either
 *         the source has been clean for SOURCE_STEADY_US or it has stalled so
 *         long (SOURCE_GIVE_UP_US) that refusing to play is the worse failure.
 */
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

/**
 * @brief Publish a new origin. Every unit on the floor anchors to what this
 *        sets.
 *
 * @param target  The instant the next packet's audio is due, us.
 */
static void start_timeline(int64_t target)
{
    /* Past the gate, so this call will reach the msg.restart assignment and
     * the flag has been delivered. Cleared here rather than where `recovered`
     * is read, so an early return above leaves it set for the next packet. */
    s_underrun_recover = false;
    next_play_at = target;
    /* The carry described the timeline this call replaces. Same reason
     * s_trim_owed is reset at a playback start in both play.c files: a
     * fraction banked against a position nothing occupies is not owed. */
    s_play_at_rem = 0;
    /*
     * local_start is assigned at the END of streamer_send_sbc(), not here.
     *
     * It has to be the stamp of the audio that will actually be at position
     * zero of the ring, and that is NOT this packet: sbc_in decodes and feeds
     * before calling in, so this packet's PCM went into the ring a moment ago
     * and the reset below is about to throw it away. Position zero belongs to
     * the NEXT packet, one packet's worth of audio later.
     *
     * The queue entry record_phase_point() makes already dates position zero
     * that way, so setting local_start to `target` here would put the two in
     * disagreement -- this speaker starting on the next packet's audio at the
     * previous packet's instant, which the phase servo then reads as a real
     * error and spends a long time walking out.
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

/**
 * @brief Keep the existing timeline near real time: slew, or jump if it is
 *        past saving.
 *
 * Bring it back by SLEWING, not by jumping. A jump steps every unit's phase by
 * the whole error at once, and each unit then servos it off with a loop built
 * for parts-per-million of drift -- minutes of every speaker being audibly in
 * a different place. A step is also the wrong description of the fault: the
 * usual cause is a pause in delivery, which reduces the playback lead by
 * exactly its own length, and rebuilding that lead is real work the servo has
 * to do either way. It can do it smoothly if the timeline moves smoothly.
 *
 * TIMELINE_SLEW_US bounds the rate to what the servo can follow: it may trim
 * up to RATE_TRIM_MAX_HZ, so a slew near that rate would leave the units
 * unable to keep up and put the error straight back.
 *
 * @param now     Arrival instant of this packet, us.
 * @param target  Where the timeline would be if it were stamped from `now`.
 */
static void steer_timeline(int64_t now, int64_t target)
{
    const int64_t err = next_play_at - target;

    if (llabs(err) > RESYNC_HARD_US) {
        /*
         * Held, not jumped, while the local ring is starving. See
         * TIMELINE_HOLD_STARVE_MS: a starved ring manufactures this error all
         * by itself, because next_play_at stops advancing while target keeps
         * moving with `now`. Jumping re-stamps the fleet to fix delivery, and
         * each jump arms a boundary every unit splices at -- so a burst of
         * source wander becomes a burst of audible splices, with the rings
         * ballooning behind the inserts. Held, the same excursion recovers on
         * its own the moment the delivery burst arrives.
         *
         * Held means held: no jump, no s_jump_arm, nothing but the one log
         * line. If the error is still past saving when the ring refills, the
         * jump below takes it on the next packet; if the burst has unwound it,
         * nothing was re-stamped and no unit ever spliced. s_slewing is left
         * exactly as it was.
         *
         * TIMELINE_HOLD_GIVE_UP_US bounds a hold that never ends, though the
         * cleaner escape usually arrives first: a ring starved all the way to
         * empty ends in CHUNK_UNDERRUN, s_underrun_recover and
         * start_timeline() -- a wholesale restart every satellite re-anchors
         * on, rather than a step in the one that exists.
         */
        const int32_t level_ms = (int32_t)((int64_t)(LOCAL_RING_BYTES -
                xStreamBufferSpacesAvailable(local_ring)) * 1000
                / ((int64_t)sample_rate * AUDIO_CHANNELS * 2));
        const bool starving = level_ms < TIMELINE_HOLD_STARVE_MS;
        bool held = false;
        if (starving && (!s_hold_since ||
                         now - s_hold_since < TIMELINE_HOLD_GIVE_UP_US)) {
            /* One line per episode, not one per packet: this branch is reached
             * on every send for as long as the starvation lasts. */
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
             * Jump, and say so. */
            next_play_at = target;
            s_play_at_rem = 0;      /* same reason as in start_timeline() */
            s_slewing = false;
            /*
             * Tell the satellites, which nothing used to do. A jump lands on
             * every unit as a step in the stamps it is receiving, with no
             * splice hint and -- being below PHASE_INSANE_US -- no re-anchor
             * either, so each one walks it off through its servo instead.
             *
             * NOT flagged on this packet. A boundary makes each unit splice by
             * its own phase reading, and right now those readings describe the
             * timeline that just stopped existing -- the hub's own included.
             * The flag is armed and fires a few packets later, once the
             * reading it will act on was taken after the discontinuity.
             * SYNC_PHASE_HIST packets is the natural delay: it is how many
             * readings the splice's median window holds, so waiting that long
             * is waiting for the window to be clear of the jump.
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

            /* Only worth reporting if it PERSISTS. err oscillates with the
             * source's burst pattern, so the trough routinely crosses the
             * threshold and recovers with the next burst about a second later;
             * announcing each of those is several alarms a minute for
             * something entirely normal. */
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

/**
 * @brief Decide what boundary flags this packet carries.
 *
 * Two different events wear the same flag on the wire, and they differ in
 * whether this unit splices too.
 *
 * A TRACK BOUNDARY: every unit nulls its phase when playback reaches the
 * flagged audio, this one included. That is the case the flag was built for.
 *
 * A TIMELINE RESTART after a local underrun: the satellites did NOT restart
 * with us. They are still playing against the old timeline, and the stamps
 * they are about to receive step by however far the timeline moved, which is
 * why telling them is worth doing at all. That step is not bounded by what a
 * splice can absorb -- a satellite absorbs what it can and servos off the
 * rest. We must NOT splice: our phase was just re-anchored to zero by
 * construction, while s_phase_err_us still holds whatever it read before the
 * underrun, and acting on that would cut up to MAX_SPLICE_MS out of the first
 * audio of the new timeline for no reason.
 *
 * @param recovered  This packet is the first of a timeline restarted after a
 *                   local underrun.
 */
static void flag_boundaries(bool recovered)
{
    /* The armed hard-jump boundary, counted down in packets and fired once. It
     * becomes an ordinary track boundary at that point: every unit splices by
     * its own phase, this one included, and by now every unit's reading was
     * taken after the jump. See where s_jump_arm is set. */
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

/**
 * @brief Record where this packet's audio sits and when it is due -- the
 *        servo's input, and the analysis's interpolation anchor.
 *
 * The position was captured by streamer_begin_packet() before the audio was
 * fed; it is paired here with the time that audio is due, and the playback
 * task compares the two when it gets there.
 *
 * @param started  A timeline start happened on this packet, so the captured
 *                 position describes an origin that no longer exists.
 */
static void record_phase_point(bool started)
{
    /*
     * Skipped at a timeline start, and that matters more than it looks.
     * streamer_begin_packet() snapshots s_samples_in before this packet is
     * decoded and fed; start_timeline() then resets s_samples_in to zero and
     * clears the ring, so the snapshot describes an origin that is gone. On
     * the first start it is legitimately zero, but on an underrun restart it
     * is every frame fed since the last one -- hours of them.
     *
     * Queued, that entry would sit at the head of a queue playback reaches
     * only after playing the same hours of audio. The tail never advances, the
     * queue fills, no further points are ever recorded, and s_phase_valid
     * stays false for good -- so the servo stops for the rest of the session.
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
        /* Full. Deliberately not counted at a timeline start, where skipping
         * is correct and routine; here it means playback is not draining the
         * queue and the servo is about to stop getting input. */
        n_phase_drop++;
    }

    /*
     * The same pair, kept for the analysis, which is fed on arrival and so
     * runs ahead of the instant it is dating audio with.
     *
     * streamer_feed() cannot know this packet's play_at -- sbc_in decodes and
     * feeds before calling in, which is what assigns it. Interpolating from
     * the previous packet's pair covers that: the anchor is one packet stale
     * and the arithmetic is the same interpolation either way, so the label is
     * right to the microsecond.
     */
    s_vis_anchor_pos = s_pending_pos;
    s_vis_anchor_due = next_play_at;
}

/**
 * @brief What became of one packet's sends, so fan_out() can time the gap
 *        between packets that actually REACHED THE AIR.
 *
 * The distinction is not academic. fan_out() is CALLED on schedule whether or
 * not the sendto inside it succeeds, so a gauge that stamps on the call reads
 * perfectly even through a stretch where the hub emitted nothing at all --
 * exactly the event it exists to catch.
 *
 * NO_LISTENERS is its own answer rather than folded into REFUSED, because the
 * two mean opposite things about the link: nothing was refused, there was
 * simply nobody to send to, and an empty floor must not accumulate a gap that
 * then reads as a spike the moment a satellite joins.
 */
typedef enum {
    FANOUT_SENT,          /**< At least one sendto took it. */
    FANOUT_REFUSED,       /**< Every sendto was refused -- a real hole. */
    FANOUT_NO_LISTENERS,  /**< Nothing to send to, so nothing to measure. */
} fanout_result_t;

/**
 * @brief Unicast the built packet to each registered listener.
 *
 * Multicast was removed entirely: it is never acknowledged and never retried,
 * and lost a large share of packets at every PHY rate tried. Unicast gets
 * link-layer ACK and retransmission and measured essentially clean. Airtime
 * now scales with speaker count, which is affordable at SBC bitrates and would
 * not have been at PCM.
 *
 * @param bytes  Size of `msg` for this packet, from AUDIO_MSG_BYTES().
 * @return Whether the packet reached the air, was refused, or had no audience.
 */
static fanout_result_t send_audio_to_clients(size_t bytes)
{
    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    /* One satellite taking it is the packet reaching the air: the gauge asks
     * whether the hub emitted, not whether every listener was served, and a
     * per-listener failure is already counted by tx_fail_note_audio(). */
    bool any_listener = false, any_sent = false;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!snapshot[i].last_seen) {
            continue;
        }
        any_listener = true;
        if (sendto(sock, &msg, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {

            /*
             * One immediate retry, ENOMEM only. The pool frees buffers as
             * frames complete, well inside one audio period, so a second
             * sendto costs a syscall and may find room where the first did
             * not. It cannot be deferred: a resend arriving after a newer
             * packet is a seq-drop on the satellite. See n_audio_retry.
             *
             * The refusal is still counted when the retry also fails, so
             * tx-fail keeps meaning what it has always meant.
             */
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

/**
 * @brief Send the audio packet built in streamer_send_sbc() to whoever is
 *        listening, and time the gap since the last one that got out.
 *
 * @param bytes  Size of `msg` for this packet, from AUDIO_MSG_BYTES().
 * @param now    The instant this packet was stamped, us.
 */
static void fan_out(size_t bytes, int64_t now)
{
    /* Counted here rather than at the sendto, so it is the rate the timeline
     * slews at and the rate the TX pool sees, not the rate that happened to
     * succeed. See s_audio_pkts. */
    s_audio_pkts++;
    /* Aged first, so the snapshot below only ever skips a cleared slot rather
     * than acting on a stale one. telemetry.c ages it too, so the second age
     * is redundant but cheap. */
    clients_age(now);
    const fanout_result_t r = send_audio_to_clients(bytes);

    /*
     * How far apart two audio packets actually reached the air -- the hub's
     * half of the satellite's arrival gauges (sat.h, rx_gap_max_us).
     *
     * The satellite can see a hole in its arrivals and cannot tell whether the
     * packets were sent late or delayed on the way. This is the other end of
     * that question, and it has to be measured here because this is the only
     * place that knows when the hub let go of one. Steady state is the ~20 ms
     * of one SBC packet with the source's own lumpiness on top.
     *
     * AFTER the send, and only on success. A refused packet leaves the
     * previous stamp standing, so the next success measures the WHOLE hole --
     * which is the reading the satellite saw and this end otherwise did not.
     *
     * NO_LISTENERS resets the stamp instead: there is no hole when there is
     * nobody to send to.
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

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0

/**
 * @brief Send one completed group's parity to the same client list the audio
 *        went to.
 *
 * Its own lane, and it YIELDS. fan_out() ignores the ENOMEM backoff because
 * audio always sends -- a refused audio packet is a hole in the sound. Parity
 * is the opposite: it exists to repair holes, and a parity datagram that takes
 * the last TX buffer MAKES one. So it stands down while the pool is in
 * backoff, exactly as the frame and volume lanes do.
 *
 * That is not a corner case, it is the whole failure mode the scheme has to
 * avoid: redundancy is wanted precisely when the channel is busy, which is
 * precisely when the transmit pool has nothing to spare. Yielding here means
 * the cost of parity under contention is that it stops, not that it starts
 * displacing the audio it was meant to protect.
 *
 * @param bytes  Size of the parity datagram, from AUDIO_FEC_MSG_BYTES().
 */
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

        /* No ENOMEM retry, unlike the audio lane. A retry costs a syscall in
         * the audio path's own task, and what it would buy is a repair rather
         * than the sound itself. If the pool is that tight, the backoff above
         * is about to fire anyway. */
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

/**
 * @brief Fold one sent packet into its group's parity, and publish the parity
 *        when the group closes.
 *
 * Groups are aligned on seq -- member index is seq % FEC_K -- so the satellite
 * derives the same grouping from arithmetic and nothing has to be negotiated
 * or carried on the audio packets. audio_msg_t is untouched by the whole
 * scheme, which is what keeps a satellite that predates it reading the stream
 * unchanged.
 *
 * @param m    The packet just handed to fan_out().
 * @param now  Unused; the grouping is derived from seq, not from time.
 */
static void fec_note_sent(const audio_msg_t *m, int64_t now)
{
    (void)now;
    if (!s_fec) {
        return;
    }
    const uint32_t idx = m->seq % FEC_K;

    if (idx == 0) {
        /* Clear only what the last group used, not the full ceiling: typical
         * payloads are well under a whole parity datagram, several times a
         * second. */
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

/* Claim the parity buffer: a no-op when parity is configured off, and a
 * warning -- not a fallback -- when PSRAM refuses. See s_fec for why internal
 * DRAM is not an acceptable second choice. Declared in streamer.h. */
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
    /*
     * Recovering from a local underrun. The flag cannot be raised inside
     * start_timeline(): a start clears s_restart_pending, because it throws
     * away any track boundary that was waiting for the old timeline, so the
     * request to bring the satellites with us would be set and wiped in the
     * same call.
     *
     * CONSUMED BELOW, NOT HERE. Clearing the flag on sight would let the
     * source-steadiness gate `return` before msg.restart was ever written --
     * and it does return, for up to SOURCE_STEADY_US after a hole, which is
     * exactly the condition an underrun leaves behind. The flag would be
     * consumed into a local, thrown away with the packet, and no satellite
     * would ever learn the timeline had restarted.
     *
     * So it stays set until a packet actually carries it. Re-entering here
     * while still waiting is harmless: next_play_at is already 0 and
     * `recovered` is recomputed from the flag each time.
     */
    bool recovered = false;
    if (s_underrun_recover) {
        next_play_at = 0;
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
     * How much lead this packet is going out with -- see n_lead_min_us in
     * hub.h for what it is paired against.
     *
     * Here, at the stamp, and against the `now` read at the top of this call:
     * that is the same instant the satellite's arrival lead is measured from
     * at the other end, so the two subtract to a transit time.
     */
    {
        const int64_t lead = next_play_at - now;
        /* Refused rather than clamped, exactly as the satellite's copy is: a
         * gauge that clamps reports its own rail as a measurement. This end
         * cannot see the offset step that makes that happen there -- both
         * terms are this board's own clock -- but the two ends must not
         * disagree about what a lead is, and a bound that never fires is the
         * cheap half of that. */

        if (lead <= LEAD_INSANE_US && lead >= -LEAD_INSANE_US &&
            lead < n_lead_min_us) {
            n_lead_min_us = (int32_t)lead;
        }
    }
    memcpy(msg.payload, sbc, len);

    const size_t bytes = AUDIO_MSG_BYTES(len);
    fan_out(bytes, now);

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0

    /*
     * Fold this packet into its group's parity, and send the parity if it
     * completed one. After fan_out(), so a packet that never reached the send
     * cannot be covered by a parity that was already on the air -- and so that
     * a packet the TX pool REFUSED still is: the satellite lost it either way,
     * and parity repairs a hole made at this end exactly as it repairs one
     * made on the air.
     */
    fec_note_sent(&msg, now);
#endif

    /*
     * The timeline advances by the audio actually sent, not by wall clock --
     * stamping "now + lead" each time would fold task jitter into playback.
     *
     * THE REMAINDER IS CARRIED. This runs once per packet forever, and
     * `frames` is a whole number of decoded SBC frames -- a multiple of the
     * codec's block size -- so the division by sample_rate is exact only when
     * `frames` happens to be a multiple of it, which no SBC frame count is.
     * Dropping the remainder loses a fraction of a microsecond per packet,
     * which at ~50 packets a second is tens of parts per million: the lead in
     * front of every stamp shrinks steadily, and nothing corrects it, because
     * steer_timeline() only slews past RESYNC_US and the servo's depth net
     * only clamps past its own threshold -- the drift lives entirely inside
     * both deadbands until a satellite's margin is thin enough that an
     * ordinary delivery gap empties its ring.
     *
     * Both symptoms have this sign: a next_play_at that advances slower than
     * the audio it describes carries less lead than LEAD_US to the satellites,
     * and starts this unit's own playback sooner relative to the audio fed, so
     * the local ring drains too.
     *
     * The accumulator is the same shape as the fine trim's TRIM_ONE_FRAME in
     * both play.c files, and for the same reason: a correction far below the
     * unit of the thing being corrected has to be banked until it is worth
     * one. Exact over any number of packets -- the loss becomes zero, not
     * smaller.
     */
    const int64_t advance = (int64_t)frames * 1000000LL + s_play_at_rem;
    next_play_at += advance / (int64_t)sample_rate;
    s_play_at_rem = advance % (int64_t)sample_rate;

    /*
     * Our own speaker joins the timeline, on the audio that will be at
     * position zero of the freshly reset ring -- the next packet, whose stamp
     * is the one this call has just left behind. Written here rather than in
     * start_timeline() so that it is the same expression as the timeline
     * itself, and cannot drift from it again.
     */
    if (started) {
        local_start = next_play_at;
        /* Published AFTER the instant, and it is what the play task waits on.
         * Writing the value first is the whole of the ordering guarantee --
         * see the declaration in hub.h. */

        local_epoch++;
    }
}
