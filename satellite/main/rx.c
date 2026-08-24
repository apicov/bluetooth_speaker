/*
 * Everything that happens to a datagram: demux, anchor policy, gap policy,
 * SBC decode, and the feed into the ring.
 *
 * Split out of main.c on 2026-08-12; the bodies are unchanged.
 */
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

/*
 * expect_seq and have_seq were locals of handle_audio(), which is now several
 * functions. File scope, not shared state: rx_task is the only caller of any of
 * them, so nothing new can write these and they stay out of sat.h. The wider
 * scope is the whole cost of the split.
 */
static uint32_t expect_seq;
static bool have_seq;

/*
 * The previous audio arrival, and how long the current run of near-simultaneous
 * ones has been going. Here for the same reason as the pair above: rx_task is
 * the only writer, so the published gauges in sat.h stay write-only from this
 * side and the running state nobody else needs stays out of the header.
 */
static int64_t s_prev_audio_at;
static uint32_t s_burst_run;

/*
 * The tail of the last audio actually queued, kept so a gap can be concealed
 * with something other than digital zero.
 *
 * A lost packet is ~20 ms. Filling it with zeros is correct for the TIMELINE --
 * the length has to be exact or every later sample plays early -- and wrong for
 * the EAR: the output jumps from whatever it was playing to zero and back, and
 * those two discontinuities are a click at each end. The click is most of what
 * a dropout sounds like. The missing 20 ms of music, on its own, is much less
 * objectionable than the edges around it.
 *
 * So the fill starts from the last samples that really played and fades them to
 * silence, and the audio that resumes afterwards is faded in. Nothing here
 * invents music: past the fade it is still silence, and a long gap still sounds
 * like a gap. What it removes is the two transients, which cost nothing to
 * remove and are the part that carries.
 *
 * PLC_TAIL_FRAMES is one SBC frame's worth at the usual settings, which is the
 * most the concealer ever repeats before it has faded out entirely.
 */
#define PLC_TAIL_FRAMES 128
#define PLC_FADE_FRAMES 128          /* ~2.9 ms at 44.1 kHz, both directions */
static int16_t s_plc_tail[PLC_TAIL_FRAMES * AUDIO_CHANNELS];
static uint32_t s_plc_have;          /* frames held, 0 until the first decode */
static uint32_t s_plc_fade_in;       /* frames of fade-in still owed after a gap */

/* Remember the tail of what was just queued. Called with whole frames only. */
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

/*
 * Decide whether this packet can start a stream, and start it if so.
 *
 * Returns false when the packet is refused and there is nothing more to do with
 * it -- the estimator has not settled, a reset is already pending, or the lead
 * is wrong. Every one of those was a bare `return` in the middle of a 371-line
 * function; they are the same refusals, and the comments on each are the runs
 * that produced them.
 */
static bool anchor_stream(const audio_msg_t *msg, int64_t offset, bool by_tsf)
{
    /*
     * First chunk of a stream: the only moment playback timing is decided,
     * so it must use a clock that can be trusted. Any error here is baked
     * in for the life of the stream.
     *
     * The settling wait applies to the ESTIMATOR only. Three probes produce
     * an offset but not a good one -- minimum-RTT selection needs a full
     * window to find a genuinely uncongested round trip, which costs 2.5 s
     * of silence at every stream start. TSF needs no such wait: it is not
     * built from round trips, so there is no congested sample to average
     * away, and one beacon is as good as ten.
     */
    if (!by_tsf && !sync_est_settled(&est)) {
        static bool told;
        if (!told) {
            told = true;
            ESP_LOGI(TAG, "holding playback until the clock estimate settles");
        }
        return false;
    }

    /*
     * Two refusals, both learned from one run where this loop anchored
     * eighteen times in three seconds and every one of them was doomed
     * before playback started.
     *
     * FIRST: a packet whose play_at has already passed cannot be anchored
     * to. The scheduled wait below is what buys the ring its prefill --
     * ~200 ms of audio accumulates while playback holds for its instant --
     * and a negative wait skips it entirely. The run started at "in -90 ms"
     * with `buffer 29 ms`, and every re-anchor after it read worse: -871,
     * -1022, -1234 ms. Anchoring on those produced phase readings of one to
     * 1.25 seconds, which tripped PHASE_INSANE_US, which re-anchored, which
     * reset the ring and threw away the only audio that could have fixed it.
     *
     * The cause was upstream -- the hub's transmit path was refusing sends
     * and what arrived was late and sparse -- and nothing here could have
     * fixed that. But nothing here should have amplified it either. A packet
     * that is already late is evidence about the link, not a timeline.
     *
     * SECOND: one anchor per second. A re-anchor that does not stick is
     * worse than no re-anchor, because xStreamBufferReset() below discards
     * the buffer each time. Refusing for a second parks playback for a
     * second; the alternative measured six seconds of noise.
     *
     * Both are bounded. If every packet is late for ANCHOR_GIVE_UP_US the
     * refusal itself becomes the fault -- a hub whose lead is genuinely
     * shorter than the path, or a clock offset wrong in a way TSF agrees
     * with -- and a satellite that stays silent forever on a stream it could
     * have played badly is not the better failure. Take the packet, say so
     * at ERROR, and let the servo do what it can.
     */
    /*
     * Wait for the playback task to park before touching the ring.
     *
     * A gap-triggered resync sets have_seq false immediately, so the very
     * next packet reaches here -- possibly ~20 ms later, while playback is
     * still draining the ring this is about to reset. xStreamBufferReset()
     * refuses while a task is blocked on the buffer, so the reset would
     * silently not happen while samples_in went to zero underneath it: a
     * stale ring measured against a fresh count, which reads as an insane
     * phase and costs another re-anchor to clear.
     *
     * Bounded, and cannot deadlock: the flag is cleared when the play task
     * parks, which it does within one chunk on seeing the flag, within
     * 500 ms if it is blocked on an empty ring, and unconditionally in the
     * outer loop above whichever route it took.
     */
    if (resync_request) {
        return false;
    }

    const int64_t now_local = esp_timer_get_time();
    const int64_t start_local = sync_to_local(msg->play_at, offset);
    static int64_t refuse_since;         /* first refusal of this streak */
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
        anchor_provisional = false;   /* this one had the lead it needed */
    }
    refuse_since = 0;
    last_anchor = now_local;
    anchor_at = now_local;
    resync_request = false;   /* may have been set while playback was parked */

    stream_rate = msg->sample_rate ? msg->sample_rate : 44100;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /* Same number the playback task dates its audio with below. The LEDs
     * convert that back to a sample position, so a different rate there
     * would separate the count from the timeline at the difference -- 8.8%
     * for a 48 kHz source against the 44.1 kHz this used to assume. */
    visualiser_set_rate(stream_rate);
    /* A new origin: stream_offset is about to be re-seeded, so any frame
     * already computed and waiting to be drawn is dated against a timeline
     * that stops existing on the next line. */
    visualiser_flush();
#endif
    sbc_decoder_init();
    /* The concealer's tail belongs to the stream that just ended, and the ring
     * is about to be reset -- fading the new stream's first samples up from
     * whatever the old one was playing would be a discontinuity invented where
     * there is none. Playback restarts from a clean buffer either way. */
    s_plc_have = 0;
    s_plc_fade_in = 0;
    stream_start_local = start_local;
    stream_offset = offset;
    offset_slew_last = 0;                /* re-seed the slew for this stream */
    /* The visualiser is told nothing here. Anything of its that advances on
     * its own keys off the scheduled instant carried by each chunk, which is
     * already master-clock time and identical on every unit -- so there is
     * no offset for it to be given. */
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
    /*
     * The first anchor after a rejoin, which is the one that can be built on
     * a clock estimate measured before the outage -- see wifi_down_at. The
     * age is of the NEWEST sample in the estimator's window; if it exceeds
     * the outage then every sample min-RTT selection can choose from
     * predates the drop, and this anchor is as stale as it looks.
     */
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

/*
 * Playback is running on an anchor we already know was bad. Take the first
 * packet that could have been anchored on properly and start again.
 *
 * A give-up anchor is the least-bad answer to "nothing anchorable for five
 * seconds", but it is permanent as it stands: a run anchored at -317 ms
 * lead, read phase +331 ms, and nothing ever re-anchored because that is
 * comfortably inside PHASE_INSANE_US. So the two servos were left to argue
 * about it -- the phase servo pulling the rate up to catch up, the depth net
 * pulling it down because catching up drains the ring -- and the speaker sat
 * a third of a second behind the floor for minutes.
 *
 * None of that is drift, and no rate fixes it. The lateness came from the
 * hub's timeline being displaced, and the moment the hub recovers there is a
 * packet with proper lead in front of it. Re-anchoring on that erases the
 * error in one step instead of asking a 1 ms/s loop to walk it off.
 *
 * The flag is cleared here rather than when the new anchor lands, so one
 * provisional anchor buys exactly one upgrade attempt. If the attempt runs
 * into the give-up path again it sets the flag again, and ANCHOR_MIN_INTERVAL_US
 * bounds how fast that can cycle.
 */
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

/*
 * Fill one missing packet's worth (want_frames) into the ring as concealment.
 * Advances samples_in by exactly the frames the ring accepted -- the same
 * accounting as decode_into_ring(), because every marker and phase point is
 * recorded against samples_in and an undercount would slide the whole stream.
 *
 * THIS IS THE FALLBACK NOW, NOT THE REPAIR. It used to take a recovered payload
 * and decode as much of it as had arrived, then pad the remainder -- which was
 * ~1/4 of every recovery, because a whole copy never fitted beside the packet
 * that carried it. XOR parity recovers a packet WHOLE or not at all, so a
 * repaired packet goes through decode_into_ring() like any other and this is
 * reached only when nothing could repair the loss: two losses in one group, a
 * parity that never arrived, or parity switched off.
 */
static uint32_t fill_gap_silence(uint32_t want_frames)
{
    static const int16_t silence[128 * AUDIO_CHANNELS] = {0};
    uint32_t filled = 0;

    /*
     * Concealed rather than silenced: the last audio that really played, faded
     * to zero over PLC_FADE_FRAMES, then zeros for however much gap is left.
     * Same LENGTH as the silence it replaces -- every frame is still counted
     * into samples_in below, so the timeline is untouched and this cannot slide
     * the stream. Only the content differs.
     *
     * Before the first decode there is nothing to fade from, so it degrades to
     * exactly the old behaviour.
     */
    static int16_t conceal[128 * AUDIO_CHANNELS];
    uint32_t faded = 0;                  /* frames of fade emitted so far */

    while (filled < want_frames) {
        uint32_t left = want_frames - filled;
        uint32_t n = left > 128 ? 128 : left;
        const int16_t *src = silence;

        if (s_plc_have && faded < PLC_FADE_FRAMES) {
            for (uint32_t i = 0; i < n; i++) {
                const uint32_t pos = faded + i;
                /* Linear ramp to zero; past the ramp the rest of this block is
                 * zero too, which is what the multiply gives without a branch. */
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
            break;                           /* raced the playback task; counted above */
        }
    }

    /* Whatever resumes after this gap is ramped back up, so the far edge is not
     * a step either. Armed even when nothing was concealed: the discontinuity
     * into real audio exists whether the gap was faded or silent. */
    s_plc_fade_in = PLC_FADE_FRAMES;
    return filled;
}

/*
 * Conceal `missing` lost packets of `per_frames` each, in sequence order.
 *
 * SPLIT OUT OF absorb_sequence_gap() so the two callers can differ on WHEN.
 * A gap used to be filled the instant the packet after it arrived, which is the
 * only thing that can be done with a FIFO ring -- and it is also what makes
 * parity impossible, because the parity for a group cannot have arrived yet.
 * The FEC layer below defers the decision by holding the packets behind the
 * hole; this is what it calls when the hole turns out to be unrepairable after
 * all, and what absorb_sequence_gap() calls directly when it always was.
 *
 * Counting the gap is NOT here, deliberately: n_gaps is incremented at
 * detection, once, before anything tries to repair it. That is what makes
 * `gaps` a measure of what the air lost rather than of what reached the
 * speaker, which is how the 2026-08-24 A/B proved the channel and not the FEC
 * was what fixed the run. n_fec_recovered is the other half of the pair.
 */
static bool fill_gap(uint32_t missing, uint32_t per_frames)
{
    const uint32_t frames_missing = missing * per_frames;

    /*
     * HERE, NOT AT DETECTION, and the two are no longer the same place.
     *
     * n_gaps counts what the air lost; this counts what the room heard, and
     * parity is what drove them apart -- a repaired gap never reaches this
     * function, so it contributes nothing to the silence total. Counted at
     * detection, as it used to be, `gaps 40 (800 ms silence)` would claim 800 ms
     * of holes on a run where 38 of the 40 came back whole.
     *
     * Above the resync guards below, exactly where it was relative to them, so
     * every path that reached the old counter still reaches this one and no
     * window's figure changes meaning for a reason other than the repair.
     */
    n_gap_frames += frames_missing;
    {
        /*
         * Past a point, filling the gap is the wrong answer.
         *
         * Silence of exactly the right length is right for one or two lost
         * packets: it costs 20 ms of audio and keeps this speaker's position in
         * the timeline, which is the whole reason the fill exists. It stops
         * being right when the gap is an OUTAGE rather than jitter, because the
         * silence has to go somewhere and the ring is not sized for it.
         *
         * Measured, at a stream start where the hub's transmit path dropped ~98
         * datagrams stepping from idle to ~135 a second: the satellite filled
         * 443 ms of silence across twelve gaps, into a ring only 78 ms deep. It
         * came out at `buffer 400 ms` against a 464 ms cap, with ring-full 31 --
         * and a 400 ms ring IS playing late, so phase read +172 ms and the servo
         * spent FIVE MINUTES and nine retunes draining it. The anchor had been
         * perfect: 175 ms of lead, playback started +1 us. All of the damage came
         * from the fill.
         *
         * So beyond GAP_RESYNC_MS this asks for a re-anchor instead. That costs
         * one clean stop and the wait for a packet with proper lead -- a few
         * hundred ms of silence, once -- against minutes of a speaker sitting
         * audibly behind the floor. Both are silence; only one of them ends.
         *
         * have_seq is dropped so the next packet enters the anchor path, where
         * ANCHOR_MIN_LEAD_US decides when playback may start again. The playback
         * task is told separately, because it is mid-stream and will otherwise
         * keep draining a ring that is about to be reset under it.
         */
        if (frames_missing > (uint32_t)((uint64_t)GAP_RESYNC_MS * stream_rate / 1000)) {
            n_gap_resyncs++;
            resync_request = true;
            have_seq = false;
            return false;
        }
        /*
         * samples_in must count this silence, and the old loop did not.
         *
         * It is the position every marker and every phase point is recorded
         * against, so audio that goes into the ring uncounted puts all of them
         * one packet (~20 ms) too early -- permanently, and again on the next
         * loss. The playback task then measures its phase against the wrong
         * packet and the servo obediently holds the speaker at that error, so
         * each lost packet moved this unit ~20 ms away from the hub and it never
         * came back.
         *
         * Nothing showed it: the marker pulse fires off the same skewed count,
         * so it lands where the hub's does while the sound and the lights slide.
         *
         * The tail below 128 frames is now inserted too, for the same reason:
         * dropping it left up to 2.9 ms uncounted per loss.
         *
         * Asked of the ring first, rather than discovered 128 frames at a time.
         * A gap is worth `missing` whole packets and a packet is ~20 ms, so a
         * burst loss asks for more than the ring holds: one run's ten-packet
         * gap wanted 8960 frames -- 203 ms, against a 200 ms target depth and
         * the 372 ms ring of the time -- and got 5120 of them in. (RING_TARGET_MS
         * is 250 now and RING_BYTES is 80 kB, so that particular gap would fit;
         * a longer one still would not, and the arithmetic below is what makes
         * the shortfall honest either way.) Pushing until it jams then
         * breaking out reached the same place, but it did so through seventy
         * failing sends, and the shortfall came out as a number nobody could
         * check against the ring's actual free space at the time.
         *
         * WHAT THE SHORTFALL MEANS IS NOW ACTED ON, which it was not. The note
         * that used to end here said the owed frames "are not in it, so playback
         * runs that much early until the servo walks it back", and the servo
         * cannot walk it back: it trims +-100 Hz, 2.27 ms/s, against a shortfall
         * that arrives hundreds of milliseconds at a time.
         *
         * Measured 2026-08-13 17:53. A hub transmit burst cost 37 audio packets
         * in one window; the satellite met them with
         *
         *   gaps 18 (658 ms silence) | ring-full 15
         *   gaps 13 (920 ms silence, 3 short by 229 ms) | ring-full 63
         *
         * -- 658 ms of silence written into a 464 ms ring in a few hundred ms of
         * wall time, because a gap is filled the moment the packet AFTER it
         * arrives rather than at the rate audio plays. The ring jammed, 229 ms
         * went uncounted, and the timeline slid by exactly that. The unit then
         * sat at `phase +250217 us` with `buffer 409 ms` for over two minutes,
         * hunting with +-97 Hz retunes it had no chance of closing, and only
         * escaped via "no anchorable packet for 5005 ms".
         *
         * So a fill that does not fit is treated as a gap too big to fill, which
         * is the same fault by a different route and already has the right
         * answer: reset and re-anchor. That costs a few hundred ms of silence
         * once. The slide costs minutes, and takes the unit out of sync with
         * every other speaker on the floor while it lasts.
         *
         * Counted separately from n_gap_resyncs so the two causes stay
         * distinguishable -- a gap longer than GAP_RESYNC_MS says the air was
         * bad, this says the ring could not absorb the burst.
         */
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
        /*
         * Fill in sequence order, one packet's worth at a time, so the timeline
         * length stays exact whatever the ring does mid-fill.
         */
        uint32_t filled = 0;
        for (uint32_t i = 0; i < missing && filled < frames_missing; i++) {
            uint32_t per = frames_missing - filled;
            if (per > per_frames) {
                per = per_frames;
            }
            uint32_t got = fill_gap_silence(per);
            filled += got;
            if (got < per) {
                /*
                 * The ring jammed mid-fill despite having had room a moment ago
                 * -- the space check above is a snapshot and this task is not
                 * the only one touching the buffer. Same consequence as the
                 * check failing outright, so the same answer: the frames owed
                 * to the timeline are not in it, and re-anchoring is the only
                 * way back that does not leave the unit permanently early.
                 */
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
/* Started below; declared here because the gap path is what arms it. */
static bool fec_hold_begin(const audio_msg_t *msg, uint32_t missing);
#endif

/*
 * A lost packet must become silence of exactly the right length -- or, if parity
 * can still repair it, the packet itself a little later. Skipping it would pull
 * every later frame earlier and slide the whole stream against the master: a
 * permanent error, not a momentary glitch.
 *
 * Returns false when this packet must not be delivered: it was filled for, it
 * was a duplicate, or it has been taken into the FEC hold and will be delivered
 * when the group resolves.
 */
static bool absorb_sequence_gap(const audio_msg_t *msg)
{
    if (have_seq && msg->seq > expect_seq) {
        const uint32_t missing = msg->seq - expect_seq;
        n_gaps++;

        /*
         * Offered to parity FIRST, and the gap is already counted, so a repaired
         * loss still reads as a loss on the air. If the hold takes it, this
         * packet is not delivered now -- it is delivered in seq order behind the
         * repaired one when the parity lands, a few tens of milliseconds from
         * here and still ~310 ms ahead of when it plays.
         */
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
        if (fec_hold_begin(msg, missing)) {
            return false;
        }
        /*
         * Parity would not even take it on -- most often because the gap is
         * more than one packet, which one parity per group cannot cover by
         * construction. Counted here so that, per window,
         *
         *     gaps == fec + fec-lost
         *
         * holds exactly: every gap the air made either came back whole or did
         * not, and there is no third bucket for a loss nothing tried to repair.
         * Without this line a burst loss would vanish from both columns and the
         * repair rate would read better than it was.
         */
        n_fec_lost++;
#endif
        return fill_gap(missing, msg->frames);
    } else if (have_seq && msg->seq < expect_seq) {
        /*
         * A duplicate or a reorder, dropped -- its audio slot has already been
         * filled with silence or a recovered copy, so there is nowhere to put
         * it. Counted now because the two possible causes want opposite
         * responses and nothing could tell them apart.
         *
         * Duplicates should be impossible: the hub sends each packet once, and
         * the link layer retries below the socket rather than above it. If this
         * ever moves, something is duplicating on the air or in lwIP. Reordering should be
         * near-impossible too on a single-hop link with one traffic class -- and
         * if it is happening, the gap that preceded it was not a loss at all,
         * so the silence filled for it was inserted against a packet that did
         * arrive, just late. That is a fill this unit should stop doing, and
         * this is the number that would justify the work.
         */
        n_seq_dropped++;
        return false;
    }
    return true;
}

/*
 * Where this packet's audio lands, and when it is due.
 *
 * All three of these record a position against samples_in and must run before
 * the decode below moves it.
 */
static void record_packet_positions(const audio_msg_t *msg)
{
    /* Tag before queuing, so the mark lands at the start of this packet's
     * audio and travels through the buffer with it. */
    if (msg->marker && marker_sample < 0) {
        marker_sample = samples_in;
    }

    if (msg->restart && restart_pos < 0) {
        restart_pos = samples_in;
    }

    /* Same idea, for every packet: remember where this audio lands and when it
     * is due, so playback can measure its own phase on arrival. */
    uint32_t next = (phase_head + 1) % PHASE_Q_LEN;
    if (next != phase_tail) {
        phase_q[phase_head].pos = samples_in;
        phase_q[phase_head].play_at = msg->play_at;
        phase_head = next;
    } else {
        n_phase_drop++;      /* see the counter: this used to be silent */
    }
}

/* Turn the packet's SBC into PCM and hand it to the ring. */
static void decode_into_ring(const audio_msg_t *msg)
{
    static int16_t pcm[SBC_MAX_PCM_SAMPLES];

    /* Decode here rather than at the hub: that is the entire point of sending
     * SBC, and it costs a quarter of the airtime. */
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    uint32_t pkt_frames = 0;      /* frames of this packet already queued */
#endif
    size_t off = 0;
    while (off < msg->payload_len) {
        size_t consumed = 0, samples = 0;
        if (!sbc_decode_frame(msg->payload + off, msg->payload_len - off,
                              &consumed, pcm, &samples)) {
            /*
             * The rest of this packet is dropped and the decoder is reset. This
             * one DOES reset -- unlike the FEC path, which must not -- because
             * a failure on the live stream means the decoder's own state is
             * suspect, and that is what a resync is for.
             *
             * Counted because it is a hole in the audio that no other counter
             * sees: the frames are simply never queued, so samples_in does not
             * advance for them and the timeline shortens by however much the
             * packet had left. The hub distinguishes a CRC failure from any
             * other (dec vs dcrc on its sbc_in line); this end is one number,
             * because on this side the interesting question is only whether it
             * is happening at all.
             */
            n_decode_err++;
            sbc_decoder_init();              /* resync rather than wedge */
            break;
        }
        if (consumed == 0) {
            break;
        }
        off += consumed;
        size_t want = samples * sizeof(int16_t);

        /*
         * Ramp back up if a gap just ended. The concealer faded the near edge
         * down; this is the far edge, and without it the step from silence into
         * full-level audio is the second of the two clicks a dropout makes.
         *
         * In place in pcm[], before the ring sees it and before the visualiser
         * is fed, so both get the samples that will actually be heard.
         */
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

        /* Keep the tail for the next gap, taken from the audio as it will be
         * heard -- after any fade-in above, not before. */
        plc_note(pcm, (uint32_t)(samples / AUDIO_CHANNELS));

        /* Count what the ring actually took, not what we offered: the same
         * accounting the gap filler above depends on, and a short send here
         * would otherwise bias every later position the other way. */
        size_t sent = xStreamBufferSend(ring, pcm, want, 0);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
        /*
         * The same audio, dated the same way, fed to the analysis here rather
         * than at the DAC -- see write_audio() for why that moved.
         *
         * play_at is this packet's first sample; interpolate across the frames
         * already taken from it. That is exactly the pairing recorded in the
         * phase queue above, so the LEDs and the phase measurement are reading
         * one timeline rather than two.
         *
         * Fed what the ring TOOK, so the count the block grid rides on stays
         * equal to the audio that will actually be played. Offering what was
         * dropped would separate them at every short send.
         */
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


/* Defined below the FEC layer, which calls it to release a repaired or held
 * packet. One definition, three callers -- see the note on it. */
static void audio_deliver(const audio_msg_t *msg);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
/*
 * XOR parity, receiving end.
 *
 * THE PROBLEM THIS SOLVES IS NOT ARITHMETIC, IT IS ORDER. A loss is discovered
 * when the packet AFTER it arrives, and the parity that could repair it is not
 * sent until the group's last packet -- so at the moment the old code filled the
 * hole with silence, the repair had not been transmitted yet. The ring is a FIFO
 * and a hole cannot be back-filled once audio has been written past it.
 *
 * So the packets behind an unrepaired hole are HELD, not decoded, until the
 * group resolves one way or the other. The hold is bounded by the group: at
 * worst the loss is member 0 and the parity arrives after member K-1, which is
 * (K-2) packet times -- 40 ms at K=4, against a 350 ms hub lead and a 350 ms
 * target ring depth. The audio still reaches the ring with ~310 ms in hand, far
 * above the 125 ms ANCHOR_MIN_LEAD_US floor, and the ring dips by 40 ms out of
 * 557. That budget is why K is 4 and the ceiling is 8; see the Kconfig help.
 *
 * IT DEGRADES TO EXACTLY THE OLD BEHAVIOUR. Two losses in one group, a parity
 * that never arrives, a hub built with a different K, a payload too long to
 * cover -- every one of them ends in fill_gap(), which is the silence fill that
 * would have happened anyway, just up to (K-2) packet times later. Nothing here
 * can put wrong audio into the ring: audio_fec_extract() refuses any recovery
 * whose rebuilt header is not exactly the packet that went missing.
 */
_Static_assert(CONFIG_DANCEFLOOR_AUDIO_FEC_K >= 2 &&
               CONFIG_DANCEFLOOR_AUDIO_FEC_K <= AUDIO_FEC_K_MAX,
               "DANCEFLOOR_AUDIO_FEC_K must be 0 (off) or 2..AUDIO_FEC_K_MAX");
#define FEC_K        CONFIG_DANCEFLOOR_AUDIO_FEC_K
#define FEC_HOLD_MAX (FEC_K - 1)      /* a loss at member 0 holds the rest of the group */

/*
 * The group's running XOR, and the packet a repair rebuilds into.
 *
 * ONE ACCUMULATOR, NOT A WINDOW OF K PACKETS. Recovering member i needs every
 * other member of the group, and the ones before the hole have already been
 * decoded and thrown away -- but XOR does not need them individually. Folding
 * each arrival into a single buffer as it passes, then folding the parity in on
 * top, leaves exactly the codeword of whatever never came. That is the whole
 * repair, in AUDIO_FEC_CODEWORD_MAX bytes instead of K times that.
 *
 * s_fec_rec is a byte array cast to audio_msg_t rather than an audio_msg_t: the
 * struct is packed, so there is nothing to align, and its payload[] ceiling is
 * AUDIO_MAX_PAYLOAD (2048) where parity can only ever cover
 * AUDIO_FEC_PAYLOAD_MAX (1438). The same cast rx_task makes on its receive
 * buffer, for the same reason.
 */
static uint8_t  s_fec_acc[AUDIO_FEC_CODEWORD_MAX];
static uint8_t  s_fec_rec[AUDIO_FEC_CODEWORD_MAX];
static uint16_t s_fec_span;          /* bytes of s_fec_acc[] in use */
static uint32_t s_fec_base;          /* seq of member 0 of the open group */
static uint8_t  s_fec_seen;          /* bit per member index that arrived */
static bool     s_fec_open;          /* a group is being accumulated */
static bool     s_fec_ok;            /* ...and it can still produce a repair */
static uint32_t s_fec_closed_base;   /* the last group whose parity was handled */
static bool     s_fec_closed;

/*
 * The packets waiting behind an unrepaired hole.
 *
 * Stored on the wire as they arrived -- AUDIO_MSG_BYTES(payload_len) bytes, not
 * decoded PCM, which would be four times the size and would have to be undone
 * if the repair succeeded.
 */
static struct {
    bool     active;
    uint32_t seq;                    /* the single seq that did not arrive */
    uint32_t frames;                 /* what it was worth, for the fallback fill */
    uint32_t last_seq;               /* newest seq held, for contiguity */
    int64_t  began_at;               /* local us, for fec_hold_max_us */
    int      n;
    uint16_t len[FEC_HOLD_MAX];
    uint8_t  buf[FEC_HOLD_MAX][AUDIO_FEC_CODEWORD_MAX];
} s_hold;

static void fec_group_reset(uint32_t base)
{
    /* Only what the last group used. At ~851-byte payloads that is ~877 bytes
     * rather than the full 1464 ceiling, twelve times a second. */
    memset(s_fec_acc, 0, s_fec_span);
    s_fec_span = 0;
    s_fec_base = base;
    s_fec_seen = 0;
    s_fec_open = true;
    s_fec_ok = true;
}

/* A group is finished with -- repaired, given up on, or clean. Marked rather
 * than merely forgotten, so the packets delivered out of the hold afterwards do
 * not fold themselves into a fresh accumulator for a group that is over. */
static void fec_group_close(void)
{
    if (s_fec_open) {
        s_fec_closed_base = s_fec_base;
        s_fec_closed = true;
    }
    s_fec_open = false;
}

/*
 * Fold one delivered packet into its group's running XOR.
 *
 * Called for every packet that reaches the ring, in the one place that puts
 * them there -- so a held packet is folded in when it is HELD (the parity may
 * arrive before it is flushed, and the XOR has to be complete by then) and the
 * closed-group guard above is what stops it being folded in twice.
 */
static void fec_note_arrival(const audio_msg_t *m)
{
    const uint32_t base = m->seq - (m->seq % FEC_K);

    if (s_fec_closed && base == s_fec_closed_base) {
        return;                      /* already accounted for; group is over */
    }
    if (!s_fec_open || base != s_fec_base) {
        fec_group_reset(base);
    }
    if (!audio_fec_xor_in(s_fec_acc, &s_fec_span, m)) {
        s_fec_ok = false;            /* payload longer than parity can cover */
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

/*
 * How long the hold lasted, folded into the window's maximum.
 *
 * Called at BOTH resolutions, because a hold that ends in silence costs the ring
 * exactly what one that ends in a repair does -- the packets were not written
 * either way. Measuring only the successful ones would report the cost of the
 * cases that worked and hide the cases that waited just as long for nothing.
 */
static void fec_hold_note_duration(void)
{
    const int64_t held = esp_timer_get_time() - s_hold.began_at;

    if (held > 0 && held < INT32_MAX && (int32_t)held > fec_hold_max_us) {
        fec_hold_max_us = (int32_t)held;
    }
}

/* Deliver what was held, in the order it arrived. The repaired packet (or the
 * silence that stood in for it) has already gone in ahead of them. */
static void fec_hold_flush(void)
{
    for (int i = 0; i < s_hold.n; i++) {
        audio_deliver((const audio_msg_t *)s_hold.buf[i]);
    }
    s_hold.n = 0;
}

/*
 * The hole is not going to be repaired: fill it and release what was waiting.
 *
 * This is the path every failure ends on, and it lands in exactly the place the
 * receiver would have been without parity at all -- the same fill_gap(), of the
 * same length, counted the same way. The only difference is that it happened up
 * to (K-2) packet times later, which the lead absorbs.
 */
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
        /* fill_gap asked for a re-anchor: the ring is about to be reset under
         * us and these packets have nowhere to go. Dropping them is not a loss
         * of position -- have_seq is already false, so the next packet re-enters
         * the anchor path and the timeline is rebuilt from it. */
        s_hold.n = 0;
    }
}

/*
 * Whether a packet may be held or delivered at all.
 *
 * BOTH HALVES, and they are not the same condition. have_seq says this receiver
 * knows where it is in the sequence; stream_start_local says there is a timeline
 * to play against. Only rx.c clears the first, but play.c zeroes the second on
 * its own -- on an underrun and on a resync -- without touching have_seq at all.
 *
 * A hold that outlived that would resolve into a stream that no longer exists:
 * fec_hold_abandon() would write concealment and flush its held packets into a
 * ring the anchor path is about to reset, and if anchor_stream() then REFUSED
 * the packet -- which it does while resync_request is set, while the estimator
 * settles, and through an ANCHOR_MIN_LEAD_US refusal streak -- into a ring
 * nothing resets at all. The stale audio would be discarded soon enough; the
 * phase points pushed beside it carry a play_at from before the stall, and the
 * playback task measures against those.
 *
 * So the hold's lifetime is exactly the anchor gate's, and this is the one place
 * that says so. It is deliberately the same test handle_audio() applies two
 * lines further down before calling anchor_stream().
 */
static bool fec_stream_ready(void)
{
    return have_seq && stream_start_local != 0;
}

/* Everything the FEC layer knows, thrown away. Called when the stream is not in
 * a state a hold can resolve into: the group it was accumulating belongs to a
 * timeline that no longer exists. */
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

/*
 * A gap has just been detected. Hold behind it instead of filling it, if parity
 * could still repair it.
 *
 * Every one of these refusals means "the silence fill is the right answer now",
 * and they are the whole of the latency argument: the hold may only last until
 * the parity for the group the hole is in, which has not been sent yet.
 */
static bool fec_hold_begin(const audio_msg_t *msg, uint32_t missing)
{
    const uint32_t want = expect_seq;      /* the seq that did not arrive */

    if (missing != 1) {
        return false;                      /* parity repairs one loss, not two */
    }
    if (anchor_provisional) {
        /* Holding would defer upgrade_provisional_anchor() past the packets it
         * is waiting for, and an anchor known to be bad is worth more than one
         * repaired packet. */
        return false;
    }
    if (msg->payload_len > AUDIO_FEC_PAYLOAD_MAX) {
        return false;                      /* this group will get no parity */
    }
    if (want / FEC_K != msg->seq / FEC_K) {
        return false;                      /* the group's parity is already past */
    }
    if (!s_fec_open || !s_fec_ok || s_fec_base != want - (want % FEC_K)) {
        return false;                      /* nothing accumulated to repair from */
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

/*
 * A packet arrived while a hold is in flight. Take it if it belongs behind the
 * same hole; otherwise say so and let the caller give up.
 */
static bool fec_hold_offer(const audio_msg_t *m)
{
    if (m->seq != s_hold.last_seq + 1) {
        return false;                      /* a second loss in the same group */
    }
    if (m->seq / FEC_K != s_hold.seq / FEC_K) {
        return false;                      /* group over, parity did not come */
    }
    if (m->payload_len > AUDIO_FEC_PAYLOAD_MAX) {
        return false;
    }
    return fec_hold_push(m);
}

/* Rebuild the one missing member and deliver it, then everything behind it. */
static bool fec_repair(const audio_fec_msg_t *fm, uint32_t want_seq)
{
    for (uint16_t i = 0; i < fm->span; i++) {
        s_fec_acc[i] ^= fm->parity[i];
    }
    /* The parity is at least as long as the longest member, so the codeword now
     * extends to its span -- and the next group's reset must clear that far. */
    s_fec_span = fm->span;

    if (!audio_fec_extract(s_fec_acc, fm->span, want_seq,
                           (audio_msg_t *)s_fec_rec)) {
        n_fec_bad++;
        return false;
    }

    /*
     * COUNT THE LOSS IF NOTHING ELSE DID.
     *
     * When the member that went missing was the group's LAST one, the parity
     * arrives before any audio packet behind it, so absorb_sequence_gap() never
     * sees a seq jump and never counts the gap. Repairing it silently would make
     * `gaps` a count of losses the repair happened to be too slow for -- which
     * is the opposite of what it is for. It has to mean what the air lost,
     * before and independently of what was done about it, or a comparison
     * between a parity run and a bare one measures the parity twice.
     *
     * s_hold.active is the test because it is exactly the record of whether a
     * gap was already counted: a hold only ever starts from a detected gap.
     */
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

/*
 * One parity datagram. This is where a repair happens -- including for a loss
 * nothing has noticed yet, when the member that went missing was the group's
 * last and no audio packet has arrived behind it to reveal the gap.
 */
static void fec_parity_rx(const audio_fec_msg_t *fm, int n)
{
    n_fec_parity_rx++;

    /*
     * The second gate, and it is not reachable from the first: a parity can
     * repair a loss and deliver a packet without any audio datagram passing
     * through handle_audio() in between, so the check there does not cover this
     * path. Same condition, same reason -- fec_repair() ends in audio_deliver(),
     * and that must not write into a stream that is not there.
     */
    if (!fec_stream_ready()) {
        fec_reset();
        return;
    }

    /*
     * Trusted only if all of it agrees: the sender's K, a span that covers at
     * least a bare header and no more than a codeword, a datagram long enough to
     * hold what it claims, and the group this receiver actually accumulated.
     * A hub and a satellite built with different K fail the count or the base
     * and are counted, rather than combining unrelated packets.
     */
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
            fec_group_close();       /* a clean group: nothing to repair */
            return;
        }
        /* Exactly one bit set. Two losses XOR to something that is not a packet,
         * and audio_fec_extract() would refuse it anyway -- but there is no
         * point building it to find out, and n_fec_lost is the honest counter
         * for a group parity cannot help with. */
        if ((lost & (uint8_t)(lost - 1u)) == 0) {
            const uint32_t want = s_fec_base + (uint32_t)__builtin_ctz(lost);
            if (want == expect_seq && fec_repair(fm, want)) {
                return;
            }
        }
    } else if (fm->count != FEC_K) {
        n_fec_bad++;                 /* the two ends disagree about K */
    }

    fec_hold_abandon();
    fec_group_close();
}
#endif  /* CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0 */

/*
 * Everything a packet does once the sequencing has accepted it.
 *
 * Pulled out of handle_audio() because there are three callers now, not one: a
 * packet that arrived in order, a packet the parity rebuilt, and a packet that
 * was held behind a hole while the parity was on its way. All three must land
 * identically -- same marker handling, same phase point, same samples_in
 * accounting -- and the surest way to guarantee that is for there to be one
 * copy of it.
 *
 * That is the deeper reason parity recovers the audio_msg_t HEADER as well as
 * the payload. The scheme it replaces produced a bare buffer of SBC, which had
 * to be fed through a separate fill routine that reimplemented a subset of this
 * and got the accounting subtly different. A recovered packet here is a packet.
 */
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

/*
 * One audio packet, in the order the decisions have to be taken.
 *
 * Each step either finishes the packet or hands it on. That sequence was the
 * shape of the 371-line original too -- it was just spelled as early returns
 * between four screens of commentary, so the order could not be read without
 * reading all of it.
 */
static void handle_audio(const audio_msg_t *msg, int64_t arrived_at)
{
    int64_t offset;
    bool by_tsf = false;
    if (!clock_offset(&offset, &by_tsf)) {
        return;                              /* clock not trusted yet, discard */
    }

    /*
     * How much of the hub's LEAD_US survived the trip -- see the note on
     * rx_lead_min_us in sat.h, which is where the reasoning lives.
     *
     * Here rather than in rx_task because this is the first line that has an
     * offset it is allowed to trust. A lead measured against an untrusted clock
     * is a number about the clock, not about delivery, and it would land in the
     * gauge indistinguishable from a real collapse.
     *
     * Before every early return below, so a packet refused by the anchor or the
     * gap logic still says when it got here. Those are exactly the packets a
     * delivery hole produces.
     */
    const int64_t lead = msg->play_at - (arrived_at + offset);
    if (lead > LEAD_INSANE_US || lead < -LEAD_INSANE_US) {
        n_lead_insane++;        /* see LEAD_INSANE_US: refused, not clamped */
    } else if (lead < rx_lead_min_us) {
        rx_lead_min_us = (int32_t)lead;
    }
    if (msg->format != AUDIO_FMT_SBC) {
        ESP_LOGW(TAG, "unexpected audio format %u -- hub and satellite disagree",
                 msg->format);
        return;
    }

#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
    /*
     * Before the anchor, not after, and that ordering is load-bearing.
     *
     * A hold that gives up may ask for a re-anchor (fill_gap can), and this
     * packet is then the one that has to carry it. Deciding the hold first
     * leaves have_seq in its final state before anchor_stream() reads it, so
     * the packet lands in exactly one of the two paths rather than being
     * sequenced against a stream that is about to be rebuilt.
     *
     * The test is fec_stream_ready() rather than have_seq alone, and it must
     * stay the same test the anchor gate below applies -- see the note on it.
     */
    if (!fec_stream_ready()) {
        fec_reset();          /* nothing a hold could resolve into */
    }
    if (s_hold.active) {
        if (fec_hold_offer(msg)) {
            return;           /* held; delivered when the group resolves */
        }
        fec_hold_abandon();   /* it belongs to a later group: the hole is real */
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
    /* Sized for the largest message we can receive, which is audio -- a parity
     * datagram is capped at the MTU and so is comfortably smaller. */
    static uint8_t buf[sizeof(audio_msg_t)];
    _Static_assert(sizeof(buf) >= AUDIO_UDP_MTU, "a full datagram would not fit");

    while (1) {
        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        int64_t t4 = esp_timer_get_time();
        if (n < 1) {
            /*
             * A zero-length datagram is nothing to do; an error is not.
             *
             * This used to `continue` on both, silently. recvfrom() blocks, so
             * in the normal case the loop is idle -- but an error does NOT
             * block, and a persistent one (the socket going down under a WiFi
             * drop, most plausibly) turns this into a spin at priority 7 that
             * nothing counts and nothing bounds. It would starve the very
             * receive path it is failing to serve, and present as loss on the
             * air.
             *
             * Counted, and yielded on. One tick is nothing against a real
             * datagram rate of ~136 a second, and it is the difference between
             * a spin and a poll.
             */
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
            /* The window carries no timestamps of its own, and the whole
             * question about a rejoin is how old the newest sample in it is. */
            est_newest_at = t4;
        } else if (buf[0] == MSG_VOL && n >= (int)sizeof(vol_msg_t)) {
            const vol_msg_t *v = (const vol_msg_t *)buf;
            /* Clamped rather than trusted: a byte past full scale would build a
             * gain that amplifies instead of attenuating. */
            const uint8_t vol = v->volume > AUDIO_VOL_MAX ? AUDIO_VOL_MAX
                                                          : v->volume;
            /* Before the change test below, so repeats count too -- see n_vol_rx
             * in sat.h for why the repeats are the interesting half. */
            n_vol_rx++;
            /* Repeated once per telemetry window, so say something only when it
             * moved -- this is the audio path's log discipline, and a line every
             * 5 s saying nothing changed is the mistake the RX counters exist to
             * undo. */
            if (vol != audio_volume || !audio_vol_known) {
                ESP_LOGW(TAG, "VOLUME %u/%d", vol, AUDIO_VOL_MAX);
            }
            /* Level first, flag second -- see sat.h. Playback may read the pair
             * between these two stores; observing the flag then guarantees the
             * level beside it, and observing neither is the state it was
             * already in. */
            audio_volume = vol;
            audio_vol_known = true;
        } else if (buf[0] == MSG_META && n >= (int)sizeof(meta_msg_t)) {
            const link_meta_t *m = (const link_meta_t *)((const meta_msg_t *)buf)->payload;
            ESP_LOGW(TAG, "TRACK #%" PRIu32 ": \"%s\" - %s [%s]",
                     m->track_id, m->title, m->artist, m->album);
        } else if (buf[0] == MSG_FRAME && n >= (int)FRAME_MSG_BYTES(0)) {
/*
 * GATED ON THE SOURCE, not just on the visualiser. It was the latter alone, and
 * on a unit doing its own analysis that meant validating every batch and
 * memcpying ~9 frames out of it, 9.8 times a second, into a
 * visualiser_submit_frame() compiled down to `(void)f`. Harmless and pure
 * waste -- and it kept n_frames_rx climbing on a unit whose strip owed the
 * number nothing, which is worse than the wasted cycles.
 *
 * The datagram still ARRIVES on a local unit: it is group-addressed, and this
 * unit is in the group for the audio. Only the parsing goes.
 */
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER && CONFIG_DANCEFLOOR_LED_SOURCE_REMOTE
            /*
             * A batch of analysis frames the hub computed. Each is drawn at the
             * instant it names, exactly like one this unit computed itself, so
             * the two sources are interchangeable -- which is what lets the hub
             * run any algorithm at all without it having to be deterministic
             * across units.
             *
             * ONE DATAGRAM, A BEACON'S WORTH OF FRAMES. The hub cannot transmit
             * to the group more often than the DTIM beacon lets it, so it fills
             * a datagram with everything the analysis produced in between rather
             * than sending one frame and dropping the rest -- which is what it
             * did until 2026-08-20, and why this unit's strip lurched while the
             * hub's followed the music. Submitted in the order they were
             * computed, because df::RemoteDetect derives flux from the
             * difference between consecutive frames and would read a shuffled
             * batch as noise.
             *
             * The shape is checked rather than assumed: `len` is one frame and
             * `count` how many follow, both against the bytes that actually
             * arrived. A hub and a satellite on different builds is the mismatch
             * this protocol is most likely to meet, and reinterpreting a frame
             * of the wrong shape would put garbage on the strip rather than
             * nothing. Said once: it is a property of the pair of builds, so it
             * will not stop happening, and ten complaints a second would bury
             * everything else.
             */
            const frame_msg_t *fm = (const frame_msg_t *)buf;
            if (fm->len == sizeof(vis_frame_t) && fm->count >= 1 &&
                (size_t)fm->count * fm->len <= FRAME_PAYLOAD_MAX &&
                n >= (int)FRAME_MSG_BYTES((size_t)fm->count * fm->len)) {
                for (unsigned i = 0; i < fm->count; i++) {
                    vis_frame_t f;
                    memcpy(&f, fm->payload + i * sizeof(f), sizeof(f));
                    visualiser_submit_frame(&f);
                }
                /* Frames, not datagrams: the rate on the HEALTH line is then
                 * directly comparable with the 86/s the hub analyses at, which
                 * is the comparison that catches this lane being throttled. */
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
            /*
             * This is the CLOCK SOURCE, not a measurement. It was one, and this
             * comment still said so long after clock_offset() started
             * preferring it for anchoring and for track_offset() -- the
             * declaration comment on tsf_offset_us is the accurate one. Read
             * everything below as feeding playback directly, because it does.
             *
             * The comparison: both units relate their own TSF to their own
             * esp_timer, and because both TSFs track the same AP counter the
             * difference of those two deltas is the clock offset -- with no
             * round trip in it, so no path asymmetry (see tsf_msg_t).
             */
            tsf_msg_t m;
            memcpy(&m, buf, sizeof(m));

            /*
             * Bracket the esp_timer read between two TSF reads, so the pair
             * carries its own error bar.
             *
             * The two reads are not atomic, and anything that preempts between
             * them lands directly in the delta. The first run showed it: the
             * steps sat at 1 to 80 us and then jumped +333, -296, +360, -355,
             * +521, -515 -- always a jump and an immediate return, which is one
             * bad sample rather than a clock moving. `span` is how long the
             * pair actually took, so a preempted sample can be discarded
             * instead of being read as TSF instability.
             */
            const int64_t tsf_a = esp_wifi_get_tsf_time(WIFI_IF_STA);
            const int64_t my_local = esp_timer_get_time();
            const int64_t tsf_b = esp_wifi_get_tsf_time(WIFI_IF_STA);
            const int64_t span = tsf_b - tsf_a;
            const int64_t my_tsf = (tsf_a + tsf_b) / 2;   /* centred on the timer read */

            /*
             * Say what the first one looked like, whatever it looked like.
             * These used to skip silently on every failure path, so a run with
             * no TSF line could not distinguish "the messages never arrived"
             * from "both counters read zero" from "the estimator was not ready"
             * -- and a measurement that fails invisibly is worse than none.
             */
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
                continue;           /* estimator not ready yet; it will be */
            }

            const int64_t tsf_offset = (m.local - m.tsf) - (my_local - my_tsf);

            /* Counted, not enforced -- see n_tsf_wide. This sample is published
             * below exactly as it was before; the counter only says how many
             * would be refused if the span were acted on. */
            if (span > TSF_SPAN_MAX_US) {
                n_tsf_wide++;
            }

            /* Published for anchoring and for the slew. This is the promotion
             * from measurement to source; everything else about the comparison
             * below stays, because it is what would show a regression. */
            tsf_publish(tsf_offset, my_local);

            /*
             * Rate-limited, and the rate limit comes FIRST so that this line
             * always appears. The previous version discarded wide-span samples
             * before reaching the log, which meant a run where every sample was
             * wide produced total silence -- and the discard counter that would
             * have explained it was only printed on the line that a surviving
             * sample was needed to reach. Twice in this experiment a diagnostic
             * has failed by staying quiet; the fix both times is that the
             * periodic line is unconditional and says what it saw.
             *
             * Nothing is discarded now. `span` is how long the read pair took,
             * so it is the error bar on this sample -- reported beside the
             * value rather than used to hide it, since whether the big steps
             * correlate with a wide span is exactly the question.
             */
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

            /* The steps are the point. The estimator swings several ms sample
             * to sample; if TSF has a hardware advantage its step should be
             * microseconds -- beacons are 102.4 ms apart and the local counter
             * free-runs at ~14 ppm between them, so ~1.4 us of drift. */
            ESP_LOGW(TAG, "TSF: est %+lld us | tsf %+lld us | diff %+lld us | "
                          "steps tsf %+lld us, est %+lld us | span %lld us "
                          "(max %lld)",
                     est_offset, tsf_offset, tsf_offset - est_offset,
                     tsf_step, est_step, span, span_max);
            span_max = 0;
        } else if (buf[0] == MSG_AUDIO && n >= (int)AUDIO_MSG_BYTES(0)) {
            /*
             * The arrival cadence, before anything can decide the packet is not
             * worth having. Counting only the packets that survive would hide
             * the case being measured: a hole is followed by a lump, and the
             * lump is where the refusals are.
             *
             * t4 is the stamp taken the instant recvfrom() returned, so this is
             * the closest to the air the receive path can get. The first packet
             * after boot has nothing to be a gap from.
             */
            n_audio_rx++;
            if (s_prev_audio_at) {
                const int64_t gap = t4 - s_prev_audio_at;
                if (gap > rx_gap_max_us) {
                    rx_gap_max_us = (int32_t)(gap > INT32_MAX ? INT32_MAX : gap);
                }
                s_burst_run = gap < RX_BURST_US ? s_burst_run + 1 : 1;
            } else {
                s_burst_run = 1;    /* the first packet is a run of one */
            }
            if (s_burst_run > rx_burst_max) {
                rx_burst_max = s_burst_run;
            }
            s_prev_audio_at = t4;
            handle_audio((const audio_msg_t *)buf, t4);
#if CONFIG_DANCEFLOOR_AUDIO_FEC_K > 0
        } else if (buf[0] == MSG_AUDIO_FEC &&
                   n >= (int)AUDIO_FEC_MSG_BYTES(0)) {
            /*
             * NOT counted into the arrival cadence above. rx_gap_max_us and
             * burst-max are about the AUDIO stream's evenness, and folding a
             * parity datagram into them would close a real gap with a packet
             * that carries no audio -- the instrument would report a delivery
             * that did not happen.
             */
            fec_parity_rx((const audio_fec_msg_t *)buf, n);
#endif
        }
    }
}
