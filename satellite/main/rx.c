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
 * Fill one missing packet's worth (up to want_frames) into the ring: a recovered
 * payload first, if a trailing FEC block carried one, then silence for whatever
 * it did not yield. Advances samples_in by exactly the frames the ring accepted
 * -- the same accounting as decode_into_ring() and the fill below, because every
 * marker and phase point is recorded against samples_in and an undercount would
 * slide the whole stream.
 *
 * Whole SBC frames are decoded until want_frames would be overshot; the rest is
 * silence, so the fill is length-exact. red == NULL means no redundancy covered
 * this packet, so the whole fill is silence -- the behaviour this path always had.
 *
 * Any shortfall is counted in n_fec_short_frames rather than quietly padded. It
 * is ~1/4 of every recovery whenever redundancy is on -- a copy of an ~825-byte
 * payload does not fit beside it -- and nothing used to say so.
 */
static uint32_t fill_recovered_then_silence(const uint8_t *red, size_t red_len,
                                            uint32_t want_frames)
{
    static int16_t pcm[SBC_MAX_PCM_SAMPLES];
    static const int16_t silence[128 * AUDIO_CHANNELS] = {0};
    uint32_t filled = 0;

    if (red && red_len) {
        size_t off = 0;
        while (filled < want_frames && off < red_len) {
            size_t consumed = 0, samples = 0;
            if (!sbc_decode_frame(red + off, red_len - off, &consumed, pcm, &samples)
                || consumed == 0) {
                /*
                 * Stop, but DO NOT reinitialise the decoder.
                 *
                 * This used to call sbc_decoder_init() here, and because the
                 * copy was cut mid-frame by the hub's MTU guard it fired on
                 * essentially every recovery. The decoder is one global context
                 * shared with the live stream, and what init() throws away is
                 * the synthesis filterbank history -- so each recovery also put
                 * a transient into the frames that followed it, on top of the
                 * silence the truncation had already caused. Two artefacts from
                 * one cause, and the second one looked like a decoder problem.
                 *
                 * A failure here says the COPY is short or corrupt. It says
                 * nothing about the live stream, whose own decode path resets on
                 * its own errors (decode_into_ring). SBC frames are
                 * independently decodable, so leaving the state alone costs
                 * nothing and keeps the history the next real frame needs.
                 */
                n_fec_decode_err++;
                break;
            }
            off += consumed;
            uint32_t f = (uint32_t)(samples / AUDIO_CHANNELS);
            if (filled + f > want_frames) {
                break;                       /* next frame would overshoot; silence the rest */
            }
            size_t want = samples * sizeof(int16_t);
            size_t sent = xStreamBufferSend(ring, pcm, want, 0);
            uint32_t got = (uint32_t)(sent / (AUDIO_CHANNELS * sizeof(int16_t)));
            filled += got;
            samples_in += (int32_t)got;
            if (sent < want) {
                return filled;               /* ring filled mid-packet */
            }
        }
    }

    /* Whatever the copy did not supply. Counted before the loop so a ring that
     * fills mid-pad does not hide the shortfall behind a second cause -- these
     * frames were owed by the recovery either way. */
    if (red && filled < want_frames) {
        n_fec_short_frames += want_frames - filled;
    }

    /*
     * The rest is concealed rather than silenced: the last audio that really
     * played, faded to zero over PLC_FADE_FRAMES, then zeros for however much
     * gap is left. Same LENGTH as the silence it replaces -- every frame is
     * still counted into samples_in below, so the timeline is untouched and this
     * cannot slide the stream. Only the content differs.
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
 * A lost packet must become silence of exactly the right length. Skipping it
 * would pull every later frame earlier and slide the whole stream against
 * the master -- a permanent error, not a momentary glitch. `frames` tells us
 * how much audio a packet was worth, so a gap can be filled accurately even
 * though SBC packets vary in size.
 */
static bool absorb_sequence_gap(const audio_msg_t *msg, int n)
{
    if (have_seq && msg->seq > expect_seq) {
        uint32_t missing = msg->seq - expect_seq;
        uint32_t frames_missing = missing * msg->frames;
        n_gaps++;
        n_gap_frames += frames_missing;

        /*
         * Trailing FEC redundancy, if the hub attached any: parse every block the
         * recv carried, so a missing packet can be decoded from the copy a later
         * packet piggybacked instead of becoming silence. Found by length, not a
         * flag, so this recovers from any sender that attached blocks and is a
         * no-op for one that did not.
         */
        struct { uint8_t ofs; const uint8_t *p; uint16_t len; } reds[8];
        int n_reds = 0;
        for (size_t off = AUDIO_MSG_BYTES(msg->payload_len);
             n_reds < (int)(sizeof(reds) / sizeof(reds[0])) &&
             off + AUDIO_RED_HDR_BYTES <= (size_t)n; ) {
            const audio_red_hdr_t *rh =
                (const audio_red_hdr_t *)((const uint8_t *)msg + off);
            if (rh->red_len == 0 ||
                off + AUDIO_RED_HDR_BYTES + rh->red_len > (size_t)n) {
                break;
            }
            reds[n_reds].ofs = rh->red_seq_ofs;
            reds[n_reds].p = (const uint8_t *)msg + off + AUDIO_RED_HDR_BYTES;
            reds[n_reds].len = rh->red_len;
            n_reds++;
            off += AUDIO_RED_HDR_BYTES + rh->red_len;
        }

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
         * Fill in sequence order. For each missing packet, decode the recovered
         * SBC a trailing FEC block carried for it -- if one did -- then pad with
         * silence so the timeline length stays exact. Partial recovery (the
         * hub's MTU guard truncated the copy) decodes to fewer frames; the
         * silence pad in fill_recovered_then_silence() makes up the difference.
         */
        uint32_t filled = 0;
        for (uint32_t i = 0; i < missing && filled < frames_missing; i++) {
            uint32_t per = frames_missing - filled;
            if (per > msg->frames) {
                per = msg->frames;
            }
            uint8_t want_ofs = (uint8_t)(msg->seq - (expect_seq + i));
            const uint8_t *red = NULL;
            uint16_t rlen = 0;
            for (int k = 0; k < n_reds; k++) {
                if (reds[k].ofs == want_ofs) {
                    red = reds[k].p;
                    rlen = reds[k].len;
                    break;
                }
            }
            if (red) {
                n_fec_recovered++;
            }
            uint32_t got = fill_recovered_then_silence(red, rlen, per);
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
    } else if (have_seq && msg->seq < expect_seq) {
        /*
         * A duplicate or a reorder, dropped -- its audio slot has already been
         * filled with silence or a recovered copy, so there is nowhere to put
         * it. Counted now because the two possible causes want opposite
         * responses and nothing could tell them apart.
         *
         * Duplicates should be impossible: the hub sends each packet once, and
         * a group frame is not retried. If this moves under multicast it means
         * something is duplicating on the air or in lwIP. Reordering should be
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

/*
 * One audio packet, in the order the decisions have to be taken.
 *
 * Each step either finishes the packet or hands it on. That sequence was the
 * shape of the 371-line original too -- it was just spelled as early returns
 * between four screens of commentary, so the order could not be read without
 * reading all of it.
 */
static void handle_audio(const audio_msg_t *msg, int n, int64_t arrived_at)
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

    if ((!have_seq || stream_start_local == 0) &&
        !anchor_stream(msg, offset, by_tsf)) {
        return;
    }
    if (upgrade_provisional_anchor(msg, offset)) {
        return;
    }
    if (!absorb_sequence_gap(msg, n)) {
        return;
    }

    expect_seq = msg->seq + 1;
    have_seq = true;

    record_packet_positions(msg);
    decode_into_ring(msg);
}


void rx_task(void *arg)
{
    (void)arg;
    /* Sized for the largest message we can receive, which is audio. */
    static uint8_t buf[sizeof(audio_msg_t)];

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
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
            /*
             * An analysis frame the hub computed. Drawn at the instant it
             * names, exactly like one this unit computed itself, so the two
             * sources are interchangeable -- which is what lets the hub run any
             * algorithm at all without it having to be deterministic across
             * units.
             *
             * The length is checked rather than assumed. A hub and a satellite
             * on different builds is the mismatch this protocol is most likely
             * to meet, and reinterpreting a frame of the wrong shape would put
             * garbage on the strip rather than nothing. Said once: it is a
             * property of the pair of builds, so it will not stop happening,
             * and 43 complaints a second would bury everything else.
             */
            const frame_msg_t *fm = (const frame_msg_t *)buf;
            if (fm->len == sizeof(vis_frame_t) &&
                n >= (int)FRAME_MSG_BYTES(fm->len)) {
                vis_frame_t f;
                memcpy(&f, fm->payload, sizeof(f));
                visualiser_submit_frame(&f);
                n_frames_rx++;
            } else {
                static bool told;
                if (!told) {
                    told = true;
                    ESP_LOGE(TAG, "frame of %u bytes, expected %u -- hub and "
                                  "satellite are not the same build",
                             fm->len, (unsigned)sizeof(vis_frame_t));
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
            handle_audio((const audio_msg_t *)buf, n, t4);
        }
    }
}
