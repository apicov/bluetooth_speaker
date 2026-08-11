#include "streamer.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "nvs_flash.h"
/* Reached under esp_netif_tcpip_exec() only -- see arp_seed(). */
#include "lwip/etharp.h"

#include "audio_out.h"
#include "sync_proto.h"
#include "visualiser.h"
#include "wifi_log.h"

#define AP_SSID   "dancefloor"
#define AP_PASS   "dancefloor"

/*
 * How far ahead of playback each chunk is stamped. Must exceed worst-case
 * network delivery -- measured RTT peaked at 28 ms, so this is a ~7x margin.
 *
 * Reduced from 250 ms so that LEAD + RESYNC stays inside the satellite's ring:
 * with bursty input the actual lead swings around this value rather than
 * sitting on it. That ring is 80 kB / 464 ms now and the bound is 350, so there
 * is room here again -- but do not spend it on the lead without a reason the
 * lead itself can serve. It buys tolerance for LATE delivery, and the delivery
 * fault this system has actually suffered was the transmit path refusing sends
 * outright, which no lead recovers. The 30 ms went to RESYNC_US below, which
 * had a measured seven-times-a-minute misfire to its name.
 *
 * The same answers "the S3 hub has PSRAM, can the lead not be 500 ms": the ring
 * a lead must fit in belongs to the SATELLITE, which is a classic ESP32 with no
 * PSRAM and a largest free block of 106 kB against the ~107 kB a 500 ms lead
 * would need. Memory on this board buys the lead nothing.
 */
#define LEAD_US   200000

/*
 * How far the presentation timeline may wander from real time before the slew
 * starts walking it back.
 *
 * Now measured rather than estimated. sbc_in tracks the longest silence between
 * audio packets and every 5 s window contains one of 79 to 112 ms, median 100 --
 * so `next_play_at - target` does not drift, it OSCILLATES: a burst arrives and
 * the timeline races ahead of real time, a gap follows and it falls behind. The
 * trough reaches -132 ms, which is past this threshold, so it trips on entirely
 * normal delivery about seven times a minute and recovers on its own within a
 * second or two.
 *
 * That is survivable now and was not before. The old code JUMPED to `target` on
 * crossing this line, writing a transient trough permanently into the timeline
 * and stepping every unit's phase by the whole amount; the slew moves 1 ms/s,
 * so a trip that lasts a second costs 1 ms and the burst refill does the rest.
 *
 * Now raised past the swing, which is what this comment used to say it wanted
 * and could not have. The blocker was never this constant: LEAD + RESYNC bounds
 * how much a satellite must buffer, and at 200 + 120 = 320 against a 372 ms
 * ring there was no room to add the 30 ms that would clear a 132 ms trough.
 *
 * The satellite's ring is 80 kB now -- 464 ms -- so the bound is 200 + 150 =
 * 350 against 464, and a swing that reaches 132 no longer touches it. What
 * should disappear from the log is the "timeline off by ... slewing back" line
 * arriving about seven times a minute on entirely normal delivery. If it still
 * does, the swing is larger than 132 on this hardware and the number to look at
 * is sbc_in's max gap, not this one.
 *
 * NOTE the ordering: a satellite still running a 64 kB ring against this must
 * buffer 350 ms of a 372 ms ring, which fits but leaves little. Flash the
 * satellite first, or together.
 *
 * The original note, still true: SBC over UART is bursty. A2DP packets arrive ~23/s, each
 * carrying ~43 ms of audio that decodes in one go, so the timeline legitimately
 * races ahead while a burst is consumed and falls behind while waiting for the
 * next -- measured swings of +-75 ms. The old 50 ms threshold fired several
 * times a second.
 *
 * The I2S link used to hide this by pacing the audio; UART does not.
 *
 * This does not affect the local ring, which is governed by rate rather than by
 * the timeline. It does set how far a satellite's start time can be off, so
 * LEAD + RESYNC must fit the satellite ring: 200 + 150 = 350 ms against 464 ms.
 */
#define RESYNC_US 150000

/*
 * Past this the timeline is not merely off, it is wrong, and no gradual
 * correction closes it in useful time. Jump instead.
 *
 * 300 ms, down from 1 s, and the number that matters is LEAD_US rather than
 * anything about the slew.
 *
 * A satellite anchors on a packet whose play_at is still ahead of it, and the
 * lead it sees is LEAD_US plus this error. So at err = -100 ms the lead has
 * already fallen to the 100 ms a satellite insists on before it will anchor at
 * all, and past roughly -200 ms packets arrive with their play_at already gone.
 * Beyond that the timeline is not merely inaccurate, it is unusable: no
 * satellite can start, and any satellite already playing is being asked to
 * absorb an error far outside the +-100 Hz its servo can trim.
 *
 * Which is what made 1 s wrong. A displacement of ~380 ms was measured sitting
 * below it and therefore slewing: the hub's own phase read +281 ms and walked
 * back at exactly the 0.9 ms/s the slew delivers, taking about five minutes,
 * with its local ring starved to 46 ms and its servo alternating 44200 / 44080
 * the whole time because a starved ring and a late phase are the same fact seen
 * twice. The satellite meanwhile refused 237 consecutive packets, gave up, and
 * anchored 317 ms late. Nothing was being protected by correcting that
 * gradually, because nothing downstream was still working.
 *
 * The slew keeps everything it was right about. Between RESYNC_US and this, the
 * error is small enough that satellites can still anchor and still follow, so
 * moving smoothly is worth more than moving fast -- see the long note above
 * TIMELINE_SLEW_US, and the -126 ms excursion that argued for it. This only
 * changes where "gradual" stops being a kindness.
 */
#define RESYNC_HARD_US 300000

/*
 * How steadily the source must be delivering before a timeline may start.
 *
 * A timeline start publishes an origin every unit anchors to, and then playback
 * begins LEAD_US later. That only works if the source can hand over LEAD_US of
 * audio in LEAD_US of wall clock. Nothing checked whether it could.
 *
 * Measured, at a track start: `timeline start` fired on `pkts 12 | eff 2070 Hz`
 * with a 1.76 SECOND hole in the SBC input behind it. Playback began, drained,
 * and underran 700 ms later; the recovery restarted the timeline, and what came
 * out the far side was displaced 338 ms -- which every satellite then saw as
 * packets arriving past their play_at, refusing 237 in a row and anchoring 317
 * ms late. The whole cascade came from starting a timeline on a source that was
 * not yet running.
 *
 * STALL is what separates a real hole from the ordinary burst pattern. A2DP
 * arrives in lumps of ~43 ms and sbc_in's `max gap` sits at 110-150 ms in every
 * healthy window, so 300 ms is clear of normal and far below the 1.76 s that
 * mattered.
 *
 * STEADY is how long it must go without one. 500 ms is more than twice LEAD_US,
 * so a source that manages it can fill the ring before playback reaches it.
 *
 * GIVE_UP bounds the wait, for the same reason the satellite's anchor guard has
 * one: a source that stalls forever is not fixed by refusing to play it, and a
 * hub that stays silent through a whole track is the worse failure. Say so and
 * start anyway.
 */
#define SOURCE_STALL_US    300000
#define SOURCE_STEADY_US   500000
#define SOURCE_GIVE_UP_US 5000000

/*
 * How far the timeline moves per packet while slewing back to real time.
 *
 * At ~50 packets/s this is 1 ms/s. The bound that matters is the servo's: it
 * trims at most +-100 Hz, which is 2.27 ms/s at 44.1 kHz, so a slew near that
 * outruns the units it is supposed to be leading.
 */
#define TIMELINE_SLEW_US 20

/* Local playback ring. The master delays its own audio by LEAD_US exactly like a
 * satellite, otherwise it would play ahead of every other speaker. Must hold the
 * lead (~21 kB) with headroom; 64 kB is 371 ms at 44.1 kHz stereo. */
#define LOCAL_RING_BYTES (64 * 1024)

static const char *TAG = "stream";

static StreamBufferHandle_t local_ring;
static i2s_chan_handle_t i2s_tx;
static int sock = -1;
static volatile uint32_t sample_rate = 44100;
static uint32_t tx_rate = 44100;         /* what the DAC clock is actually set to */
static uint32_t rate_ema;                /* smoothed measured input rate */

static void retune_dac(uint32_t hz);

#if CONFIG_DANCEFLOOR_ENABLE_MARKER
static volatile int64_t s_marker_at;      /* local time we last pulsed */
#endif

/*
 * Ring position at which a tagged packet's audio begins, or -1 for none
 * pending. Playback pulses when it reaches this, so the pulse tracks the audio
 * through the buffer rather than being predicted from a clock.
 */
/*
 * 32-bit, not 64: read by the playback task while the receive task writes them,
 * and a 64-bit load is two instructions here -- a torn read gives a garbage
 * position and a wild marker. int32 holds 13 hours of frames at 44.1 kHz.
 */
static volatile int32_t s_marker_sample = -1;
static volatile int32_t s_samples_in;     /* frames written into local_ring */

/*
 * Phase tracking, matching the satellite.
 *
 * This unit publishes the timeline, so it should hold itself to it. Servoing on
 * buffer depth alone matches its playback RATE to its input but lets its
 * POSITION wander -- and since it wanders independently of every satellite, the
 * speakers separate even though each one's buffer looks perfectly stable.
 */
#define PHASE_Q_LEN 32
typedef struct {
    int32_t pos;
    int64_t play_at;
} phase_pt_t;

static phase_pt_t s_phase_q[PHASE_Q_LEN];
static volatile uint32_t s_phase_head, s_phase_tail;
static volatile int32_t s_phase_err_us;   /* + = playing late */
static volatile bool s_phase_valid;
/*
 * The last few raw readings, for the splice. Play task only -- pushed in the
 * crossing loop, read and reset in the splice, reset at the top of the outer
 * loop -- so no volatile and no lock. See sync_phase_hist_t for why the splice
 * needs its own filter and cannot borrow the servo's.
 */
static sync_phase_hist_t s_phase_hist;
/* Set when a splice steps the phase, so the shadow average in ring_monitor_task
 * forgets a history that describes the situation before it -- the satellite has
 * done this since "Forget the phase average after a splice". */
static volatile bool s_phase_stepped;
static volatile bool s_restart_pending;   /* flag the next packet */
/*
 * Set when local playback underruns. The play task then waits for local_start,
 * which is only assigned at a timeline start -- so without this the hub stays
 * silent until the stream stops and restarts, discarding every incoming byte in
 * the meantime (seen as fed-drop climbing to the full stream rate).
 *
 * Recovery restarts the timeline, which also re-anchors every satellite, so the
 * system comes back aligned rather than merely audible.
 */
static volatile bool s_underrun_recover;
static volatile int32_t s_restart_pos = -1;

/* Never splice more than this in one go -- a larger error means something a
 * splice will not fix, and 150 ms is audible even at a track change. */
#define MAX_SPLICE_MS 150

/* Held across a DAC retune, and the play task parks on it.
 * i2s_channel_write() returns immediately once the channel is disabled, so
 * without this the play task spins through the ring at memory speed -- a
 * measured 54 ms correction cost 177 ms of buffer. */
/* True while the timeline is being walked back to real time -- see the slew in
 * streamer_send_sbc(). */
static bool s_slewing;
static bool s_slew_told;          /* whether this episode has been announced */
static int64_t s_slew_since;      /* when it started, for the 5 s filter */

static volatile bool retuning;
static volatile int64_t local_start;   /* master-clock instant local playback begins */

void streamer_set_sample_rate(uint32_t hz)
{
    if (!hz) {
        return;
    }
    sample_rate = hz;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /* The LEDs convert between the timeline and a sample position, and this is
     * the rate the other half of that conversion uses: `sample_rate` is what
     * stamps every packet and what the playback task interpolates due_us with.
     * They have to be the same number. */
    visualiser_set_rate(hz);
#endif
    /* Smooth it: single windows carry ~0.3% noise, and every retune glitches
     * audio. The servo below wants a stable baseline, not the latest sample. */
    rate_ema = rate_ema ? (rate_ema * 3 + hz) / 4 : hz;

    /*
     * Match the DAC clock to the measured input rate.
     *
     * Transmitting at 44100 while receiving 42400 drains the playback buffer at
     * ~7 kB/s -- the 250 ms of audio is gone in five seconds and never recovers.
     * That is a 4% mismatch: 40000 ppm, against the ~14 ppm crystal drift M6 is
     * designed for. No sample-level correction can absorb it; the clocks have to
     * agree.
     *
     * Matching is right whether the deficit is a genuinely slower source or lost
     * frames: either way this board only receives `hz` frames per second, so
     * playing them at `hz` is what keeps real time.
     *
     * 1% threshold: measurement noise is ~0.3%, and retuning glitches audio.
     */
    /* Big initial mismatch (44100 nominal vs ~42600 actual) is corrected once,
     * immediately. Everything finer is left to the servo, which uses the buffer
     * level rather than the noisy rate estimate. */
    if (i2s_tx && (hz > tx_rate + tx_rate / 100 || hz < tx_rate - tx_rate / 100)) {
        retune_dac(hz);
    } else {
        /* Only when it moves. This ran every 5 s and printed the same 44100
         * every time -- a twelfth of the console spent saying nothing changed. */
        static uint32_t told_hz;
        if (hz != told_hz) {
            told_hz = hz;
            ESP_LOGI(TAG, "sample rate %" PRIu32 " Hz", hz);
        }
    }
}

/* Bytes dropped because pcm_stream was full. Silent loss here looks exactly
 * like a starving ring, which is why it needs a counter. */
static volatile uint32_t s_feed_dropped;
static volatile uint32_t s_tx_fail;      /* sendto() rejections */

/*
 * Cumulative totals for a long run, never reset -- deliberately separate from
 * the per-window counters above, which are cleared every 5 s.
 *
 * A rate tells you what is happening now; a total tells you whether something
 * has been happening slowly for an hour. Nothing here had one, so the longest
 * evidenced session was seven minutes and a leak or a slow decay would have
 * been invisible. This project's own lesson: every real fault was invisible
 * until something counted it.
 */
static volatile uint32_t n_underruns;     /* local playback ran dry */
static volatile uint32_t n_restarts;      /* timeline restarted */
static volatile uint32_t n_splices;       /* track-boundary corrections applied */
static volatile uint32_t n_retunes;       /* DAC clock changes that succeeded */
static volatile uint32_t n_retunes_bad;   /* refused or failed */
static volatile uint32_t n_sta_left;      /* satellites disassociating */
static volatile uint32_t hw_play;         /* stack headroom, sampled in-task */
static volatile uint32_t hw_mon;

/*
 * Heap, with a timestamp -- because the all-time figure could not provide one.
 *
 * esp_get_minimum_free_heap_size() is a watermark since boot. A run that ended
 * at 2040 bytes free, against 21892 on two clean runs before it, said only that
 * the hub had nearly died at some point in five minutes: not when, not during
 * what, and not whether it was the same moment as the underrun on the same line.
 * The heap is one large block plus scraps -- `largest` reads 26624 in every log
 * ever captured here -- so reaching 2040 means that block went in one piece, and
 * the only thing on this unit that takes that much on demand is WiFi's 32
 * dynamic TX and 32 dynamic RX buffers. Attributing it needs a window, not a
 * watermark.
 *
 * Sampled every 5 s and cleared by each HEALTH line, so a dip lands in a named
 * minute beside sta-left, restarts and underruns. A dip shorter than the sample
 * interval is still missed; that is what the allocation hook below is for.
 */
static volatile uint32_t heap_min_window = UINT32_MAX;

/*
 * Allocation failures, which until now were silent.
 *
 * At 2040 bytes free something very likely failed to allocate, and nothing said
 * so -- it surfaced as an underrun, which names the symptom and not the cause.
 * CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS would panic instead, which is too
 * violent for a dance floor; this counts and reports.
 *
 * The hook records and does not log; see the constraints where it is defined.
 * ring_monitor_task notices within 5 s from a context where logging is safe.
 */
static volatile uint32_t n_alloc_fail;
static volatile uint32_t alloc_fail_size;   /* the largest request that failed */
static volatile uint32_t alloc_fail_caps;

/*
 * Phase points dropped because s_phase_q was full -- the only loss path in this
 * file with no counter. It is worth one, because a wedged phase queue is not a
 * degradation here but a stop: the ring servo runs on nothing but points coming
 * off this queue, and it has already been wedged once for a whole session by a
 * stale entry queued across a timeline restart. Every log line read normally
 * throughout.
 */
static volatile uint32_t n_phase_drop;

/*
 * Ring reads that came back short of a chunk, and the frames of silence padded
 * in to cover them. Suspected, not established -- see the satellite's copy for
 * the full argument. The pad is played but was never in the ring, while
 * samples_played advances by a whole chunk regardless, so if this fires at all
 * it is a permanent displacement of every later phase point.
 */
static volatile uint32_t n_short_reads;
static volatile uint32_t n_short_frames;

/*
 * How much audio goes into the DMA before a write first blocks.
 *
 * The satellite has had this since a retune was found to be costing tens of
 * milliseconds; the hub never did, and the hub is the unit with the larger
 * startup offset. i2s_channel_write() does not block while descriptors are
 * free, so on an empty channel the first writes return at memory speed:
 * samples_played advances by the whole DMA depth (6 x AUDIO_FRAMES = 34.8 ms
 * at 44.1 kHz) against a wrote_at that has barely moved, and every phase
 * reading dated inside that window is measured against a reference the DAC is
 * not pacing.
 *
 * The channel is empty at every playback START as well as after a retune -- the
 * play task was parked, so it drained. Nothing guards it there on either unit.
 * This is very likely the "-42 ms (hub), -26 ms (satellite)" startup phase in
 * clock-sync.md §8: a 16 ms difference between two units, on a cold start, on
 * both at once, taking ~45 s to walk off. On a reconnect only the satellite
 * restarts, against a hub already servoed to zero, which is why that case
 * behaves so much better.
 *
 * MEASUREMENT ONLY. Nothing is withheld and nothing is corrected; this says how
 * big the window is so a guard can be sized rather than guessed.
 */
#define REFILL_FAST_US 1000     /* below this, the write did not block */
static bool s_refill_active;
static int32_t s_refill_frames;
static const char *s_refill_why = "start";

/*
 * Satellites are sent audio by UNICAST, not multicast.
 *
 * Group-addressed frames are never acknowledged and never retried, so any
 * corrupted frame is simply lost. Measured 20% loss across three different PHY
 * rates -- the rate was never the problem, the absence of retries was. Unicast
 * gets link-layer ACK and retransmission, which is what makes 802.11 reliable.
 *
 * The cost is that airtime scales with speaker count. At ~42 kB/s of SBC that is
 * affordable for a handful of units; it would not have been for 179 kB/s of PCM.
 *
 * Registration is implicit: satellites already send time probes every 250 ms, so
 * anything that has probed recently is listening.
 */
#define MAX_CLIENTS 8
/*
 * How long a satellite that has stopped probing stays on the send list.
 *
 * Was 10 s, which is 40 probes at PROBE_PERIOD_MS -- far more tolerance than
 * losing probes needs, and the window during which this unit keeps unicasting
 * audio and 86 analysis frames a second at a station that is not there. Each of
 * those sends takes a 1152-byte DMA buffer the driver cannot deliver and will
 * not free until it gives up, and the pool is finite: pulling a satellite mid-
 * track produced 124 failed allocations over exactly ten seconds, free heap down
 * to 4580 bytes and the largest block from 26624 to 1216, recovering the instant
 * this timeout expired. An earlier instance of the same thing cost an underrun.
 *
 * 2 s is 8 consecutive probes, which is still generous against a link measured
 * essentially clean, and cuts that window five-fold. Not shorter: a satellite
 * forgotten in error gets no audio until its next probe re-registers it, and at
 * ~150 ms of ring against a 250 ms probe period that is an underrun rather than
 * a glitch. The event handler below is what makes a clean disassociation instant
 * regardless; this bound is for the satellite that vanishes without saying so.
 *
 * What remains after both is 3 to 5 failed allocations per disconnect taken MID-
 * TRACK, and that is the floor rather than a leftover to chase. Those are frames
 * handed to the WiFi driver before client_gone() ran -- about 20 buffers
 * outstanding by the same heap arithmetic that gave the 124 above -- and nothing
 * above the driver can reach a buffer it has already taken. Audio is unaffected:
 * underruns stay 0 across it. A run showing 3-5 here is this working, not
 * regressing.
 *
 * Mid-track is load-bearing in that sentence. A satellite that leaves while
 * nothing is playing costs zero, because the residual IS the in-flight sends and
 * there are none: a disassociation observed 110 s before the first audio came
 * through reported alloc-fail 0, with the heap window never below 25 kB against
 * the 1748 B the exhaustion left. Zero here is an idle disconnect, not a better
 * floor, and reading it as one would hide the case that matters.
 */
#define CLIENT_TIMEOUT_US 2000000

typedef struct {
    struct sockaddr_in addr;
    int64_t last_seen;
} client_t;

static client_t s_clients[MAX_CLIENTS];
static portMUX_TYPE s_clients_lock = portMUX_INITIALIZER_UNLOCKED;
static esp_netif_t *s_ap_netif;           /* for the MAC -> IP lookup below */
static volatile uint32_t n_sta_dropped;   /* forgotten on the event, not the timeout */
static volatile uint32_t n_sta_nolease;   /* ... and the times the lookup could not say who */
/*
 * ... and the ones the timeout got, which nothing counted.
 *
 * This is the only path by which a satellite leaves the send list without
 * incrementing anything, and it is the path an ungraceful departure takes: the
 * AP notices inactivity far later than CLIENT_TIMEOUT_US, so a unit that loses
 * power or walks out of range is dropped here and raises no event. A HEALTH line
 * therefore read identically whether a satellite had vanished mid-track and come
 * back or nothing had happened at all, which made a disconnect test
 * uninterpretable after the fact -- the counters could not say whether a quiet
 * run meant the departure was handled well or was never seen.
 */
static volatile uint32_t n_sta_timeout;

static void client_seen(const struct sockaddr_in *from)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_clients_lock);
    int free_slot = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients[i].last_seen &&
            s_clients[i].addr.sin_addr.s_addr == from->sin_addr.s_addr) {
            s_clients[i].last_seen = now;
            portEXIT_CRITICAL(&s_clients_lock);
            return;
        }
        if (!s_clients[i].last_seen && free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot >= 0) {
        s_clients[free_slot].addr = *from;
        s_clients[free_slot].last_seen = now;
    }
    portEXIT_CRITICAL(&s_clients_lock);
}

/*
 * The send list's ARP entries, added and removed alongside it.
 *
 * These exist because registering a satellite at DHCP-assign time without them
 * is actively worse than not registering it at all, which is what e6f03d1 did
 * and why 501f388 reverted it: tx-fail went from 0 -- its value in every log
 * ever captured from this unit -- to 161 on a clean join, plus 10 failed
 * allocations, as unicasts piled into a pending-ARP queue that drops its
 * overflow.
 *
 * The reason is in the DHCP server, not in chance. dhcpserver.c adds a static
 * ARP entry of its own so it can unicast the DHCPACK, and calls
 * etharp_remove_static_entry() on it BEFORE invoking the callback that raises
 * IP_EVENT_ASSIGNED_IP_TO_CLIENT. The event therefore fires at the one instant
 * the entry is guaranteed absent. Nothing repopulates it until the station
 * itself transmits -- its ARP request for this unit, which immediately precedes
 * its first probe. That is exactly why registering on a probe works, and it is
 * the whole of what registering on the DHCP reply was missing.
 *
 * So seed it. The event carries the MAC beside the address, so this needs no
 * lease lookup and cannot misidentify the station.
 *
 * Via esp_netif_tcpip_exec(): etharp_add_static_entry() asserts
 * LWIP_ASSERT_CORE_LOCKED() and CONFIG_LWIP_TCPIP_CORE_LOCKING is not set in
 * this build, so a direct call from the event task is wrong however well it
 * appears to work.
 *
 * A static entry never ages out, and the ungraceful-departure path below cannot
 * remove one because it never learns the address. That leak is bounded by the
 * DHCP pool rather than by reconnect count -- re-adding an address overwrites
 * its own slot -- so it is at most MAX_CLIENTS (8) against ARP_TABLE_SIZE (10).
 * It is also harmless: the send list is what gates sending, not the ARP table,
 * and a client that has been dropped is not transmitted to whatever the table
 * says about it.
 */
typedef struct {
    ip4_addr_t ip;
    struct eth_addr mac;
} arp_seed_t;

static esp_err_t arp_add(void *ctx)
{
    arp_seed_t *s = ctx;
    return etharp_add_static_entry(&s->ip, &s->mac) == ERR_OK ? ESP_OK : ESP_FAIL;
}

static esp_err_t arp_drop(void *ctx)
{
    return etharp_remove_static_entry((const ip4_addr_t *)ctx) == ERR_OK ? ESP_OK : ESP_FAIL;
}

/*
 * Put a satellite on the send list as soon as it has an address, rather than
 * when it next probes.
 *
 * Symmetric with client_gone(), and it exists because that function made a
 * momentary link bounce more expensive than it used to be: a disassociation
 * followed by a rejoin 13 ms later has been seen here, and dropping the client
 * then waiting for re-registration costs up to PROBE_PERIOD_MS of silence
 * against a satellite ring holding ~150 ms.
 *
 * The honest account of what this buys is narrower than that, though. lwIP does
 * not flush the ARP cache when a station disassociates, so the 13 ms rejoin
 * probably resolved from a surviving entry all along. The case this actually
 * repairs is the cold join -- no disconnect anywhere in the run -- which is
 * where e6f03d1 produced its 161 tx-fails.
 *
 * The port is not guessed: satellites bind SYNC_PORT, so it is the source port
 * of every probe and therefore what client_seen() would have recorded anyway.
 *
 * If the ARP entry cannot be seeded, this registers nothing and lets the probe
 * do it a quarter-second later. Degrading to the behaviour that has always
 * worked beats degrading to the one that was reverted.
 */
static void client_joined(const uint8_t mac[6], const esp_ip4_addr_t *ip)
{
    arp_seed_t seed;
    seed.ip.addr = ip->addr;
    memcpy(seed.mac.addr, mac, sizeof(seed.mac.addr));

    if (esp_netif_tcpip_exec(arp_add, &seed) != ESP_OK) {
        ESP_LOGW(TAG, "could not seed an ARP entry for " IPSTR
                      " -- leaving it to register on its next probe", IP2STR(ip));
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(SYNC_PORT),
    };
    addr.sin_addr.s_addr = ip->addr;
    client_seen(&addr);
    ESP_LOGI(TAG, "satellite " IPSTR " has an address -- on the send list, ARP seeded",
             IP2STR(ip));
}

/*
 * Forget a satellite the instant it disassociates, rather than when it stops
 * probing.
 *
 * The send list is keyed by IP and the WiFi event carries a MAC, so the DHCP
 * server's lease table bridges them -- the API takes the MAC in and writes the
 * IP out. If the lookup fails, or the lease is already gone, this does nothing
 * and CLIENT_TIMEOUT_US still applies: the effect is only ever to forget sooner,
 * never to forget something else, because the MAC is authoritative.
 *
 * Worth having on top of a shorter timeout because it removes the window
 * entirely for the case that actually happens -- a satellite being reflashed or
 * restarted, which disassociates cleanly. The timeout covers the case this
 * cannot see at all: a unit that loses power or walks out of range.
 */
static void client_gone(const uint8_t mac[6])
{
    if (!s_ap_netif) {
        return;
    }
    /* Outside the critical section: this walks the lease table and takes its own
     * locks, neither of which belongs inside a spinlock held by the send path. */
    esp_netif_pair_mac_ip_t pair;
    memcpy(pair.mac, mac, sizeof(pair.mac));
    pair.ip.addr = 0;
    if (esp_netif_dhcps_get_clients_by_mac(s_ap_netif, 1, &pair) != ESP_OK || !pair.ip.addr) {
        /* Counted apart from a successful drop, because "the lease could not be
         * resolved" and "there was nothing on the list to remove" are different
         * facts wearing the same missing increment. The second is the ordinary
         * case for an ungraceful disconnect: the AP notices inactivity far later
         * than CLIENT_TIMEOUT_US, so the timeout has already done the work. */
        n_sta_nolease++;
        return;
    }

    bool found = false;
    portENTER_CRITICAL(&s_clients_lock);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients[i].last_seen &&
            s_clients[i].addr.sin_addr.s_addr == pair.ip.addr) {
            s_clients[i].last_seen = 0;
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_clients_lock);

    /* Unconditionally, not only when the client was still listed: the entry was
     * seeded when the address was assigned, so it outlives a client the 2 s
     * timeout removed first. Failure is not worth a line -- there is nothing to
     * remove if this station never had one seeded. */
    ip4_addr_t gone = { .addr = pair.ip.addr };
    (void)esp_netif_tcpip_exec(arp_drop, &gone);

    if (found) {
        n_sta_dropped++;
        ESP_LOGW(TAG, "satellite " IPSTR " disassociated -- dropped from the send list",
                 IP2STR(&pair.ip));
    }
}

uint32_t streamer_take_dropped(void)
{
    uint32_t d = s_feed_dropped;
    s_feed_dropped = 0;
    return d;
}

/* SBC the SPI link delivered but the WiFi ceiling refused. Should be 0 while
 * AUDIO_MAX_PAYLOAD tracks SBC_LINK_MAX_PAYLOAD; counted, not silent, because a
 * ceiling drift here would otherwise read as satellite-side gaps. */
static uint32_t n_wifi_oversize;

/* Ring position and scheduled instant of the last packet sent, so the analysis
 * -- fed on arrival, before this packet's stamp exists -- can date what it is
 * given. See where they are set. */
static volatile int32_t s_vis_anchor_pos;
static volatile int64_t s_vis_anchor_due;

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

/*
 * Capture where the next packet's audio will start, before it is fed.
 *
 * Needed because a packet's play_at is not known until streamer_send_sbc()
 * computes it, by which point s_samples_in has already advanced past that
 * packet's audio. The pair only means anything if both halves refer to the
 * same instant in the stream.
 */
static int32_t s_pending_pos;

void streamer_request_restart(void)
{
    s_restart_pending = true;
}

void streamer_send_meta(const uint8_t *meta, uint16_t len)
{
    if (sock < 0 || len > sizeof(((meta_msg_t *)0)->payload)) {
        return;
    }
    meta_msg_t msg = { .type = MSG_META };
    memcpy(msg.payload, meta, len);

    portENTER_CRITICAL(&s_clients_lock);
    client_t snapshot[MAX_CLIENTS];
    memcpy(snapshot, s_clients, sizeof(snapshot));
    portEXIT_CRITICAL(&s_clients_lock);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (snapshot[i].last_seen) {
            sendto(sock, &msg, sizeof(msg), 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr));
        }
    }
}

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
/*
 * Send one analysis frame to every listener.
 *
 * Registered as the visualiser's publisher, so it runs on the analysis task --
 * which means it must not block. sendto() on a UDP socket does not.
 *
 * Unicast, like the audio and for the same reason: group-addressed frames are
 * never acknowledged and so never retried, measured at ~20% loss here, and a
 * fifth of the frames missing is a visibly broken strip. ~5 kB/s per listener
 * against the 30-40 the audio already costs.
 *
 * A failed send costs a satellite one frame out of 43 a second. It is counted
 * with the audio's own failures rather than separately -- the interesting
 * question is whether the link is dropping things, not which kind.
 */
static void publish_frame(const vis_frame_t *f)
{
    if (sizeof(*f) > FRAME_PAYLOAD_MAX) {
        return;                              /* refuse rather than truncate */
    }
    frame_msg_t msg = { .type = MSG_FRAME, .len = (uint8_t)sizeof(*f) };
    memcpy(msg.payload, f, sizeof(*f));
    const size_t bytes = FRAME_MSG_BYTES(sizeof(*f));

    portENTER_CRITICAL(&s_clients_lock);
    client_t snapshot[MAX_CLIENTS];
    memcpy(snapshot, s_clients, sizeof(snapshot));
    portEXIT_CRITICAL(&s_clients_lock);

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
#endif

/*
 * The same, for one analyser result.
 *
 * Deliberately a copy of publish_frame() rather than a shared helper taking a
 * type and a length. The two differ in the message they build and in nothing
 * else today, but they are on different cadences and different budgets -- a
 * frame goes out 86 times a second and a slow analyser's result once -- and the
 * first thing either is likely to grow is its own rate limit. Sharing them now
 * would have to be undone then.
 *
 * A failed send costs a satellite one result. Counted with the audio's own
 * failures, like the frame path: the interesting question is whether the link
 * is dropping things, not which kind.
 */
#if CONFIG_DANCEFLOOR_PUBLISH_ML
static void publish_ml(const ml_result_t *r)
{
    if (sizeof(*r) > ML_PAYLOAD_MAX) {
        return;                              /* refuse rather than truncate */
    }
    ml_msg_t msg = { .type = MSG_ML, .len = (uint8_t)sizeof(*r) };
    memcpy(msg.payload, r, sizeof(*r));
    const size_t bytes = ML_MSG_BYTES(sizeof(*r));

    portENTER_CRITICAL(&s_clients_lock);
    client_t snapshot[MAX_CLIENTS];
    memcpy(snapshot, s_clients, sizeof(snapshot));
    portEXIT_CRITICAL(&s_clients_lock);

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
#endif

void streamer_begin_packet(void)
{
    s_pending_pos = s_samples_in;
}

void streamer_send_sbc(const uint8_t *sbc, uint16_t len, uint32_t frames, bool marker)
{
    static audio_msg_t msg;
    static uint32_t seq;
    static int64_t next_play_at;

    if (len > AUDIO_MAX_PAYLOAD) {
        n_wifi_oversize++;
        return;
    }
    if (sock < 0 || len == 0 || frames == 0) {
        return;
    }

    int64_t now = esp_timer_get_time();
    int64_t target = now + LEAD_US;

    /*
     * Is the source actually running? See SOURCE_STEADY_US. Tracked on every
     * packet, whether or not a start is pending, so the answer is already there
     * when one is.
     */
    static int64_t s_last_pkt_us;
    static int64_t s_steady_since;
    static int64_t s_wait_since;
    if (s_last_pkt_us == 0 || now - s_last_pkt_us > SOURCE_STALL_US) {
        s_steady_since = now;              /* a hole; the run starts again here */
    }
    s_last_pkt_us = now;

    /*
     * Recovering from a local underrun. The flag cannot be raised here: the
     * timeline-start branch below clears s_restart_pending, because a start
     * throws away any track boundary that was waiting for the old timeline. So
     * the request to bring the satellites with us was set and then wiped in the
     * same call, and no satellite has ever received it.
     */
    bool recovered = false;
    if (s_underrun_recover) {
        s_underrun_recover = false;
        next_play_at = 0;              /* fall into the timeline-start path */
        recovered = true;
    }

    /* A timeline start invalidates the position this packet captured, because
     * the ring it referred to is about to be cleared. */
    bool started = false;

    if (next_play_at == 0) {
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
                return;
            }
            ESP_LOGW(TAG, "source still stalling after %lld ms -- starting the "
                          "timeline on it anyway",
                     (now - s_wait_since) / 1000);
        }
        s_wait_since = 0;
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
        started = true;
        n_restarts++;
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
        /* Frames are drawn when the instant they name comes round, so anything
         * already computed is dated against the origin this line replaces. */
        visualiser_flush();
#endif
        ESP_LOGI(TAG, "timeline start");
    } else {
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
            ESP_LOGW(TAG, "timeline off by %lld us -- too far to slew, jumping", err);
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

    msg.type = MSG_AUDIO;
    msg.format = AUDIO_FMT_SBC;
    msg.payload_len = len;
    msg.seq = seq++;
    msg.sample_rate = sample_rate;
    msg.frames = frames;
    msg.marker = marker ? 1 : 0;
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

    msg.play_at = next_play_at;
    memcpy(msg.payload, sbc, len);

    size_t bytes = AUDIO_MSG_BYTES(len);

    portENTER_CRITICAL(&s_clients_lock);
    client_t snapshot[MAX_CLIENTS];
    memcpy(snapshot, s_clients, sizeof(snapshot));
    portEXIT_CRITICAL(&s_clients_lock);

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
        if (now - snapshot[i].last_seen > CLIENT_TIMEOUT_US) {
            portENTER_CRITICAL(&s_clients_lock);
            s_clients[i].last_seen = 0;
            portEXIT_CRITICAL(&s_clients_lock);
            /* Once per departure, not once per packet: the slot is cleared
             * above, so the !last_seen test at the top of the next pass skips
             * it before reaching here again. */
            n_sta_timeout++;
            continue;
        }
        if (sendto(sock, &msg, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {
            s_tx_fail++;
        }
    }

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
    }
}

/* ------------------------------------------------------------------- wifi */

/*
 * Count them, and stop sending to them.
 *
 * Counting was the original reason: a satellite dropping off is invisible
 * otherwise -- the driver logs it, but nothing accumulates it, so a link that
 * flaps once an hour over an evening looks identical to one that never does. One
 * reason=209 SA-Query disassociation has already been seen.
 *
 * Dropping it from the send list is the other half, and it was missing. The
 * counter alone left this unit unicasting audio and analysis frames at a station
 * that had gone, for a whole CLIENT_TIMEOUT_US, exhausting the WiFi driver's
 * buffer pool -- see the note there.
 */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        n_sta_left++;
        const wifi_event_ap_stadisconnected_t *ev = data;
        if (ev) {
            client_gone(ev->mac);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_ASSIGNED_IP_TO_CLIENT) {
        /* Carries the address AND the MAC outright, so unlike the departure
         * above this needs no lease lookup and cannot fail to identify the
         * station -- which is what lets client_joined() seed ARP. */
        const ip_event_assigned_ip_to_client_t *ev = data;
        if (ev) {
            client_joined(ev->mac, &ev->ip);
        }
    }
}

static void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* Kept, because client_gone() needs it to turn a disassociating station's
     * MAC into the IP the send list is keyed by. */
    s_ap_netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                       WIFI_EVENT_AP_STADISCONNECTED,
                                                       wifi_event, NULL, NULL));
    /* The arrival half. Not WIFI_EVENT_AP_STACONNECTED: a station is associated
     * before it has an address, and the send list is keyed by one. */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                       IP_EVENT_ASSIGNED_IP_TO_CLIENT,
                                                       wifi_event, NULL, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /*
     * Fields below are set explicitly rather than left at zero from the
     * initialiser: dtim_period has a documented range of 1-10 and zero is
     * invalid, and pmf_cfg left unset makes some clients unhappy during the
     * WPA2 handshake -- which surfaces as "incorrect password" rather than
     * anything that points at the real cause.
     */
    wifi_config_t wc = {0};
    strcpy((char *)wc.ap.ssid, AP_SSID);
    strcpy((char *)wc.ap.password, AP_PASS);
    wc.ap.ssid_len = strlen(AP_SSID);
    wc.ap.max_connection = 8;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wc.ap.channel = CONFIG_DANCEFLOOR_WIFI_CHANNEL;
    wc.ap.dtim_period = 1;
    /* Matches ESP-IDF's own softAP example. Setting capable=true is a deviation
     * that some clients refuse, so leave it alone. */
    wc.ap.pmf_cfg.required = false;

#if CONFIG_DANCEFLOOR_AP_OPEN
    wc.ap.authmode = WIFI_AUTH_OPEN;
    wc.ap.password[0] = '\0';
    ESP_LOGW(TAG, "AP is OPEN (no password) -- diagnostic build");
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));

    /*
     * 20 MHz, not the 40 the driver comes up with.
     *
     * Left alone, this AP negotiated HT40: the log reads `wifi:new:<11,2>` and
     * stations join as `bgn, 40D`. On channel 11 that puts the secondary at
     * channel 7, so the AP occupies roughly the whole 2.4 GHz band and collides
     * with every other network in it. Nothing here can use the width -- the
     * traffic is ~135 small datagrams a second per satellite, which is limited
     * by transmit opportunities rather than by bits per symbol, and with the
     * PHY rate pinned to 6 Mbps it cannot use it even in principle.
     *
     * So it was paying the full interference cost of HT40 for none of its
     * throughput. Halving the occupied spectrum is the cheapest thing available
     * that reduces how often the channel is busy when this unit wants it.
     *
     * This is the one part of that experiment that stayed. The rate and the
     * aggregation went back -- see sdkconfig.defaults for the measurements that
     * sent them back -- but nothing about HT20 was part of that trade: it costs
     * this traffic nothing and takes interference away. Kept on its own merits,
     * not as a leftover.
     *
     * Must follow esp_wifi_set_config(), which resets the bandwidth to the
     * default. Not asserted: it is a mitigation, not a requirement, and a build
     * that cannot set it should still stream.
     *
     * WIFI_BW20, not WIFI_BW_HT20 -- IDF 6 removed the older spelling, and the
     * classic hub will want the same name when this comes across to it.
     */
    const esp_err_t bw = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);
    ESP_LOGW(TAG, "AP bandwidth set to HT20: %s", esp_err_to_name(bw));

#if CONFIG_DANCEFLOOR_DISABLE_PMF
    /*
     * Turn off Protected Management Frames, because its Secure Association
     * teardown is disconnecting our own satellite.
     *
     * Observed: the AP starts an SA Query, the satellite does not answer six
     * attempts, and the AP disassociates it with reason 209 -- 1.7 s off the
     * network, twice in the first 65 seconds of a run. The satellite never
     * noticed: it counted zero disconnects while this unit counted two, which
     * is why sta-left exists at all.
     *
     * pmf_cfg.capable is deprecated in IDF 6 ("set to true internally"), so it
     * cannot be used to opt out. esp_wifi_disable_pmf_config() is the supported
     * way, and it must come after esp_wifi_set_config() and before
     * esp_wifi_start(). It fails on a WPA3 or WPA2/WPA3-mixed SoftAP; this one
     * is WPA2-PSK, so it applies.
     *
     * What is given up is protection of management frames -- spoofed
     * deauth/disassoc. Data stays encrypted under WPA2-PSK and the password is
     * unchanged. For a closed floor with two boards that is a poor trade
     * against losing a speaker every half minute.
     *
     * Not asserted, because it is the fix for a fault and not a requirement:
     * if a future IDF refuses it, the log says so and the link works as it
     * does today.
     */
    const esp_err_t pmf = esp_wifi_disable_pmf_config(WIFI_IF_AP);
    ESP_LOGW(TAG, "PMF disabled on the AP: %s", esp_err_to_name(pmf));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());

    /* Power save would park the radio between beacons and add tens of ms to
     * the packets whose timing we depend on. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /*
     * TX power is left at the driver default (full). It was once capped at
     * 13 dBm to stop this radio swamping the Bluetooth receiver when both shared
     * one chip; the two-chip split removed that need, and the cap did real harm
     * by denying rate adaptation the SNR margin it needs.
     *
     * The channel stays pinned: it costs nothing and helps Bluetooth's adaptive
     * frequency hopping route around us.
     */
#if CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS > 0
    /*
     * Pinning the PHY rate exists for a reason that no longer applies, and is
     * kept only as a diagnostic knob. Set the rate to 0 to leave it alone.
     *
     * It was mandatory while audio went out by multicast: group-addressed frames
     * fall back to the 1 Mbps basic rate, where a 1041-byte frame costs 8.3 ms
     * and 172 packets/s needs 1.43 s of airtime per second -- it physically
     * cannot fit, and half the audio was dropped in the queue.
     *
     * Unicast has no such fallback. It uses rate adaptation, which is what fixed
     * the 23% loss that a fixed 24 Mbps caused. Note esp_wifi_internal_set_fix_rate()
     * applies to ALL transmission on the interface, so any non-zero value here
     * now pins unicast and disables that adaptation.
     */
#if   CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS >= 24
    const wifi_phy_rate_t want = WIFI_PHY_RATE_24M;
#elif CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS >= 12
    const wifi_phy_rate_t want = WIFI_PHY_RATE_12M;
#else
    const wifi_phy_rate_t want = WIFI_PHY_RATE_6M;
#endif
    esp_err_t rerr = esp_wifi_internal_set_fix_rate(WIFI_IF_AP, true, want);
    if (rerr != ESP_OK) {
        ESP_LOGE(TAG, "could not fix PHY rate (%s); needs "
                      "CONFIG_ESP_WIFI_AMPDU_TX_ENABLED=n", esp_err_to_name(rerr));
    } else {
        ESP_LOGW(TAG, "PHY rate pinned to %d Mbps -- rate adaptation is OFF for "
                      "unicast too; set to 0 to restore it",
                 CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS);
    }
#else
    /* Said out loud, because which of the two this build is running is the
     * first thing to know when reading a tx-fail figure off the status line. */
    ESP_LOGW(TAG, "PHY rate adaptation is ON (rate not pinned)");
#endif
    ESP_LOGI(TAG, "SoftAP \"%s\" pass \"%s\" ch %d, radio at defaults",
             AP_SSID, AP_PASS, CONFIG_DANCEFLOOR_WIFI_CHANNEL);
}

static void socket_start(void)
{
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    assert(sock >= 0);

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SYNC_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    assert(bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == 0);
}

/* ------------------------------------------------------ sync measurement */

/*
 * Last cross-unit measurement, for the per-track summary below.
 *
 * A track boundary nulls phase on every unit, so cross-unit error resets there
 * and grows until the next one. That makes the reading taken just BEFORE a
 * boundary the one worth keeping: it is how far apart the speakers had drifted
 * over a whole track, which is the number to compare sessions and builds on.
 * Any other sample depends on where in the track cycle it was taken, and
 * comparing two of those produced three confident wrong diagnoses in a row.
 */
static volatile int64_t s_sync_err_us;
static volatile int64_t s_sync_at;        /* 0 = never measured */

/* This unit's own last boundary correction, for satellites to be compared
 * against when they report theirs. */
static volatile int32_t s_hub_splice_us;
static volatile int64_t s_hub_splice_at;  /* 0 = no boundary yet */

/*
 * SHADOW, acted on by nothing: what this unit's boundary correction would have
 * been had the splice used sync_phase_median() instead of the single most
 * recent phase reading. See sync_phase_hist_t.
 *
 * Reported beside the real figure on the same line so the two can be compared
 * at the same boundary, which is the only place they are comparable. Judging
 * the filter by flashing it and reading a log window is exactly the mistake
 * that produced three wrong diagnoses here; this makes the comparison
 * simultaneous instead.
 */
static volatile int32_t s_hub_splice_med_us;

/*
 * These four sit OUTSIDE the marker guard although only the marker writes
 * s_sync_*, because the splice path and probe_task read all four
 * unconditionally -- a satellite reporting its own splice over WiFi is compared
 * against them, and that path exists precisely when no marker is fitted.
 *
 * They were inside it, which meant the hub did not compile from its tracked
 * config: CONFIG_DANCEFLOOR_ENABLE_MARKER defaults to n, and the "no marker
 * fitted" branch below says in as many words that this is the normal deployed
 * case. Every build that had ever been run carried a local sdkconfig with the
 * marker switched on, so nothing showed it.
 */
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
static QueueHandle_t s_edge_q;            /* satellite edge timestamps */

static void IRAM_ATTR monitor_isr(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    BaseType_t hp = pdFALSE;
    xQueueSendFromISR(s_edge_q, &now, &hp);
    if (hp) {
        portYIELD_FROM_ISR();
    }
}

/*
 * Reports how far a satellite's audio is from this unit's, by comparing when
 * each pulsed for the same master-clock instant. This is the end-to-end number
 * the whole design exists to deliver -- everything else (clock offset, buffer
 * level, packet loss) is a means to it.
 */
static void monitor_task(void *arg)
{
    (void)arg;
    int64_t edge;
    while (xQueueReceive(s_edge_q, &edge, portMAX_DELAY) == pdTRUE) {
        int64_t mine = s_marker_at;
        if (mine == 0) {
            continue;
        }
        int64_t err = edge - mine;
        /* Markers are 2 s apart; anything near that is a missed pulse rather
         * than a sync error, and reporting it as one would mislead. */
        if (err > 500000 || err < -500000) {
            continue;
        }
        /* Kept for the track-boundary summary, which wants the last reading
         * before the splice rather than a scroll of them. */
        s_sync_err_us = err;
        s_sync_at = esp_timer_get_time();

        /* Every ~2 s is more than anyone reads. The value is kept for the
         * track-boundary summary regardless of whether this prints. */
        static int64_t last_sync_log;
        if (s_sync_at - last_sync_log >= (int64_t)CONFIG_DANCEFLOOR_LOG_PERIOD_S * 1000000) {
            last_sync_log = s_sync_at;
            ESP_LOGW(TAG, "AUDIO SYNC: satellite %+lld us (%s)", err,
                     err >= 0 ? "late" : "early");
        }
    }
}

static void marker_start(void)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MARKER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    gpio_config_t in = {
        .pin_bit_mask = 1ULL << CONFIG_DANCEFLOOR_MONITOR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    s_edge_q = xQueueCreate(4, sizeof(int64_t));
    assert(s_edge_q);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_DANCEFLOOR_MONITOR_GPIO, monitor_isr, NULL));
    xTaskCreate(monitor_task, "syncmon", 3072, NULL, 9, NULL);

    ESP_LOGI(TAG, "sync markers on GPIO %d, watching GPIO %d -- bench instrument, "
                  "nothing corrects on it",
             CONFIG_DANCEFLOOR_MARKER_GPIO, CONFIG_DANCEFLOOR_MONITOR_GPIO);
}
#endif  /* CONFIG_DANCEFLOOR_ENABLE_MARKER */

/* --------------------------------------------------- local delayed playback */

/* I2S_NUM_1 by history: port 0 used to be the slave receiver from the bridge.
 * That link is now UART, but there is no reason to move this. */
static void i2s_start(uint32_t rate)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    /*
     * One descriptor per chunk, so a write never spans two.
     *
     * i2s_channel_disable() waits for the in-flight write to release the DMA
     * queue, and a write that needs a second descriptor waits a second
     * descriptor period for it. At the default 240 against AUDIO_FRAMES of 256
     * every single write needed two, which is most of why this unit's retunes
     * were down 2-18 ms against the satellite's 2-6. Matching them makes the
     * worst case one period.
     *
     * Both units must carry this: it also sets the output pipeline latency the
     * servo absorbs at startup, and unequal depths park them at different
     * standing offsets.
     */
    chan_cfg.dma_frame_num = AUDIO_FRAMES;
    /*
     * A starved channel must go SILENT, not repeat itself. Without this, the
     * circular TX descriptors replay the last 34.8 ms forever once
     * local_play_task takes the underrun branch and parks -- see the
     * satellite's copy for the full mechanism. Both units must carry it.
     */
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_DANCEFLOOR_I2S_BCK_PIN,
            .ws   = CONFIG_DANCEFLOOR_I2S_LRCK_PIN,
            .dout = CONFIG_DANCEFLOOR_I2S_DATA_PIN,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx));
    tx_rate = rate;
    /* Compare this against the satellite's line. The sync marker fires when a
     * chunk is written, not when it is heard, so unequal output buffering shows
     * up as a fixed offset unrelated to clock sync. */
    ESP_LOGW(TAG, "OUTPUT: I2S external DAC, buffer %d x %d frames = %d ms, "
                  "channels=%s, silence on starve",
             chan_cfg.dma_desc_num, chan_cfg.dma_frame_num,
             (int)(chan_cfg.dma_desc_num * chan_cfg.dma_frame_num * 1000 / rate),
             AUDIO_CHANNEL_MODE_NAME);
}

/*
 * Widest DRIFT correction the servo may ask for, in Hz. Deliberately not
 * applied inside retune_dac(): the initial match to the measured input rate is
 * a different thing and is legitimately several percent (44100 nominal against
 * ~42600 measured), so a bound tight enough to be useful here would refuse it.
 */
#define RATE_TRIM_MAX_HZ 100

/* What could conceivably be an audio sample rate at all. Anything outside this
 * is a broken calculation, whoever asked for it. */
#define RATE_SANE_MIN 8000
#define RATE_SANE_MAX 192000

/* What a retune costs -- see the note on the satellite's copy. The channel down
 * is measurable here; the discarded DMA buffer is not measurable anywhere in
 * software, because those frames were counted as played. */
static volatile int32_t s_retune_phase_before;
static volatile bool    s_retune_watch;
static volatile int64_t s_retune_outage_us;

/*
 * When the retune finished, and how many crossings have been narrated since.
 *
 * MEASUREMENT ONLY. The servo still withholds exactly one reading, as before;
 * these only add ages to the lines it already prints and narrate the three
 * crossings after it without withholding them.
 *
 * The question they answer is whether withholding one crossing is enough.
 * Crossings arrive one per audio packet, ~20 ms apart, and the note in the
 * crossing loop records the transient landing 1-22 ms after the retune with the
 * buffer refilling "over the following few writes" -- so a one-shot flag may be
 * covering the first third of a disturbance and handing the rest to the servo
 * as position error. Every retune then injects what the next one corrects,
 * which is the loop that pinned the trim at RATE_TRIM_MAX_HZ and ran phase to
 * +500 ms before the `retuning` park existed.
 *
 * That comment says "if the transient turns out to outlast it, the REFILL line
 * says so". It does not, for the hub -- REFILL is a satellite instrument. This
 * is the hub's version, and it sizes RETUNE_SETTLE_US directly instead of by
 * inference.
 */
static volatile int64_t s_retune_done_at;
static volatile uint8_t s_retune_tail_left;

static void retune_dac(uint32_t hz)
{
    /*
     * Nothing computed may panic the speaker. The satellite aborted on exactly
     * this path when a wrapped phase error asked for a 4.29 GHz sample rate:
     * ESP_ERROR_CHECK turned a bad number into a dead unit. A refused retune
     * costs sync, which is recoverable; an abort is not.
     */
    if (hz < RATE_SANE_MIN || hz > RATE_SANE_MAX) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "refusing a %" PRIu32 " Hz retune -- not a sample rate", hz);
        return;
    }

    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    retuning = true;
    /* Timed from here, so the 2 ms park counts: playback is stopped for it just
     * as surely as for the disable itself. */
    const int64_t down_at = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(2));        /* let the play task notice and park */

    esp_err_t err = i2s_channel_disable(i2s_tx);
    if (err == ESP_OK) {
        err = i2s_channel_reconfig_std_clock(i2s_tx, &clk);
        /* Re-enable either way: leaving the channel down stalls playback
         * silently, which looks like a dead board rather than a failed trim. */
        const esp_err_t on = i2s_channel_enable(i2s_tx);
        if (err == ESP_OK) {
            err = on;
        }
    }
    retuning = false;

    if (err != ESP_OK) {
        n_retunes_bad++;
        ESP_LOGE(TAG, "retune to %" PRIu32 " Hz failed (%s), staying at %" PRIu32,
                 hz, esp_err_to_name(err), tx_rate);
        return;
    }
    s_retune_outage_us = esp_timer_get_time() - down_at;
    n_retunes++;
    ESP_LOGW(TAG, "DAC clock retuned %" PRIu32 " -> %" PRIu32 " Hz, channel down %lld us",
             tx_rate, hz, s_retune_outage_us);
    tx_rate = hz;

    s_retune_phase_before = s_phase_err_us;
    s_retune_watch = true;
    /* Ordered after the two above: the play task reads them together and this
     * is what arms the narration. */
    s_retune_tail_left = 3;
    s_retune_done_at = esp_timer_get_time();

    /*
     * Nothing to tell the visualiser: it counts what ARRIVES, and a retune
     * disturbs playback rather than arrival. It used to need telling, on the
     * reasoning that the disable discarded the DMA buffer this task had already
     * counted as played -- retired both by moving the analysis off the playback
     * path and by the satellite's REFILL instrument showing the descriptors are
     * not discarded at all.
     */
}

/*
 * Same shape as the satellite's play task, minus the clock conversion: here
 * master time is local time. Holding the first sample until its scheduled
 * instant is what puts this speaker on the same timeline as the rest.
 */
static void local_play_task(void *arg)
{
    (void)arg;
    static uint8_t chunk[AUDIO_CHUNK_BYTES];

    while (1) {
        if (local_start == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int64_t wait = local_start - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < local_start) {
            /* spin the last stretch */
        }
        /* On the hub local time IS master time, so this is directly comparable
         * with the satellite's figure. A difference here is a difference in the
         * audio each unit is playing, which no amount of clock accuracy fixes. */
        ESP_LOGI(TAG, "local playback started: scheduled %lld, actual %lld (%+lld us)",
                 local_start, esp_timer_get_time(), esp_timer_get_time() - local_start);
        /* samples_played counts from the first sample played, which is the
         * first sample fed after the ring was reset at timeline start. Both
         * counters therefore share an origin -- do NOT reset s_samples_in here,
         * it has legitimately been counting the audio buffered during the wait. */
        int32_t samples_played = 0;
        bool was_retuning = false;    /* armed by the park below, see s_refill_active */
        /* Every reading in it was measured against the timeline this start
         * replaces, so none of them describes where this unit now is. */
        sync_phase_reset(&s_phase_hist);
        /* The channel drained while this task was parked, so it is empty here
         * for the same reason it is empty after a disable. See s_refill_active. */
        s_refill_active = true;
        s_refill_frames = 0;
        s_refill_why = "start";
        /*
         * When the DAC last accepted a chunk -- the reference the phase reading
         * is dated against. See the note in the phase loop for why it is not a
         * clock read taken there. Seeded here so the first pass, before any
         * write has happened, has a sane value rather than zero.
         */
        int64_t wrote_at = esp_timer_get_time();

        while (1) {
            if (retuning) {
                /* Do not pull from the ring while the channel is down -- writes
                 * would return instantly and drain it. */
                was_retuning = true;
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            if (was_retuning) {
                was_retuning = false;
                s_refill_active = true;
                s_refill_frames = 0;
                s_refill_why = "retune";
            }
            hw_play = uxTaskGetStackHighWaterMark(NULL);   /* only valid in-task */
            size_t got = xStreamBufferReceive(local_ring, chunk, sizeof(chunk), pdMS_TO_TICKS(500));
            if (got == 0) {
                n_underruns++;
                ESP_LOGW(TAG, "local underrun, restarting timeline");
                local_start = 0;
                s_underrun_recover = true;
                break;
            }
            if (got < sizeof(chunk)) {
                memset(chunk + got, 0, sizeof(chunk) - got);
                n_short_reads++;
                n_short_frames += (uint32_t)((sizeof(chunk) - got)
                                             / (AUDIO_CHANNELS * sizeof(int16_t)));
            }
            /* Local time IS master time here, so this is a direct read of how
             * far playback has slipped from the published timeline. */
            while (s_phase_tail != s_phase_head && samples_played >= s_phase_q[s_phase_tail].pos) {
                /*
                 * Correct for WHERE the crossing was noticed versus where it
                 * happened, which is most of this unit's phase noise.
                 *
                 * samples_played advances by AUDIO_FRAMES per iteration -- 5.8
                 * ms at 44.1 kHz -- so by the time the loop sees it has passed
                 * `pos`, it passed it up to a chunk ago, by an amount that
                 * depends on where pos falls on the chunk grid and is therefore
                 * uncorrelated sample to sample. Reading the clock here dates
                 * the crossing at "when I noticed", and the difference is pure
                 * quantisation noise on the servo's only input.
                 *
                 * Measured before this: two reads of s_phase_err_us in adjacent
                 * log lines, a millisecond apart, differing by 15.7 ms. That
                 * noise made the hub's own retune bench unmeasurable (scatter
                 * 2.9x the effect), produced a false 23 ms alarm, and is the
                 * "hub absolute phase does not settle" wart in clock-sync.md.
                 *
                 * The overshoot is known exactly, so this is arithmetic rather
                 * than a filter: writes are paced by the DAC, so the instant
                 * samples_played was `pos` is `overshoot / rate` ago.
                 */
                int32_t overshoot = samples_played - s_phase_q[s_phase_tail].pos;
                /*
                 * Capped at one chunk, because beyond that the pacing
                 * assumption is false. A splice advances samples_played by up
                 * to MAX_SPLICE_MS in a single step and those frames were
                 * discarded rather than played over time, so any point the jump
                 * carried us past cannot be dated this way. Capping leaves
                 * those readings no worse than they were before this
                 * correction existed.
                 */
                if (overshoot > AUDIO_FRAMES) {
                    overshoot = AUDIO_FRAMES;
                }
                /*
                 * Dated from when the DAC last took a chunk, not from a clock
                 * read here.
                 *
                 * samples_played describes audio handed to the DAC by the write
                 * at the END of the previous pass, and that write is the only
                 * DAC-paced event in this loop -- the same pacing the overshoot
                 * correction above already rests on. Everything between it and
                 * this line is not paced by anything: the ring receive, and
                 * whatever preemption a board also running Bluetooth, a SoftAP,
                 * SBC decode and the bridge UART hands out. Reading the clock
                 * here folds all of it into the measurement, uncorrelated pass
                 * to pass, which is the shape of the +-20 ms scatter
                 * docs/clock-sync.md §9 lists as unexplained -- two reads of
                 * s_phase_err_us a millisecond apart differing by 15.7 ms.
                 *
                 * The satellite carries the same change for symmetry, but it is
                 * not where the problem was: its load is a fraction of this
                 * one's and its readings were always the quiet ones.
                 */
                const int64_t crossed_at = wrote_at
                                         - (int64_t)overshoot * 1000000 / sample_rate;
                const int32_t err = (int32_t)(crossed_at - s_phase_q[s_phase_tail].play_at);
                /*
                 * The first reading after a retune is a transient, so it is
                 * logged and thrown away rather than handed to the servo.
                 *
                 * What a retune actually costs is not what the outage figure
                 * suggests. i2s_channel_disable() sets the channel state, then
                 * blocks on the same binary semaphore i2s_channel_write() holds
                 * across a portMAX_DELAY wait for a DMA descriptor -- and only
                 * calls handle->stop() after that. So the audio keeps playing
                 * for most of the reported outage. What stops is this task:
                 * samples_played freezes while the DMA drains and real time
                 * advances, and the next crossing reads that gap as position
                 * error. It is not one. The buffer refills over the following
                 * few writes and the position comes back on its own.
                 *
                 * Measured on the satellite bench, 19 same-rate retunes: net
                 * +4.4 ms against a 3.6 ms outage, every one positive, and the
                 * crossing landing 1-22 ms after the retune -- inside the
                 * refill every time. The servo took each of those as a real
                 * error and trimmed the rate for it, so every retune injected
                 * the disturbance the next one would correct. That is what
                 * pinned the trim at RATE_TRIM_MAX_HZ and ran phase to +500 ms.
                 *
                 * Only the first crossing is withheld. If the transient turns
                 * out to outlast it, the REFILL line says so -- a reading taken
                 * before the refill completes is the one to distrust.
                 */
                const int64_t since_retune = s_retune_done_at
                                           ? crossed_at - s_retune_done_at : -1;
                if (s_retune_watch) {
                    s_retune_watch = false;
                    ESP_LOGW(TAG, "RETUNE COST: phase %+ld -> %+ld us (net %+ld), "
                                  "channel was down %lld us -- withheld from the "
                                  "servo, crossed %lld us after the retune",
                             (long)s_retune_phase_before, (long)err,
                             (long)(err - s_retune_phase_before),
                             s_retune_outage_us, since_retune);
                } else {
                    /*
                     * Narrated but NOT withheld -- these still reach the servo
                     * exactly as they did before, so this build behaves
                     * identically and only says more. Whether they should be
                     * withheld is the question; answering it first is the point.
                     */
                    if (s_retune_tail_left) {
                        s_retune_tail_left--;
                        ESP_LOGW(TAG, "RETUNE TAIL: phase %+ld us at %lld us after "
                                      "the retune (net %+ld from before it)",
                                 (long)err, since_retune,
                                 (long)(err - s_retune_phase_before));
                    }
                    s_phase_err_us = err;
                    s_phase_valid = true;
                    sync_phase_push(&s_phase_hist, err);
                }
                s_phase_tail = (s_phase_tail + 1) % PHASE_Q_LEN;
            }

            /* Track boundary: snap phase to zero instead of letting the servo
             * walk it off over ~45 s. Only inaudible here. */
            int32_t rp = s_restart_pos;
            if (rp >= 0 && samples_played >= rp) {
                s_restart_pos = -1;
                int32_t max_frames = (int32_t)sample_rate * MAX_SPLICE_MS / 1000;
                /*
                 * Nothing measured since the last re-anchor: s_phase_err_us is
                 * whatever it read against the previous timeline, and it is not
                 * cleared. Splicing on it would cut up to MAX_SPLICE_MS of real
                 * audio to correct an error that no longer exists. Drop the
                 * boundary instead -- the servo will take out anything genuine.
                 */
                int32_t adj = s_phase_valid
                    ? (int32_t)((int64_t)s_phase_err_us * sample_rate / 1000000) : 0;
                if (adj > max_frames)  adj = max_frames;
                if (adj < -max_frames) adj = -max_frames;
                int32_t applied = 0;      /* what the splice actually moved */

                /*
                 * SHADOW: what the correction would have been on the median of
                 * the last few readings instead of the newest one. Computed
                 * here, beside the real decision and from the same history, so
                 * the two are answers to the same question at the same instant.
                 * Nothing below reads it -- the splice above still runs on
                 * s_phase_err_us. If the shadow says the median is the better
                 * number, this is the line that moves.
                 */
                int32_t med_us = 0;
                if (s_phase_valid && sync_phase_median(&s_phase_hist, &med_us)) {
                    int32_t med_adj = (int32_t)((int64_t)med_us * sample_rate / 1000000);
                    if (med_adj > max_frames)  med_adj = max_frames;
                    if (med_adj < -max_frames) med_adj = -max_frames;
                    s_hub_splice_med_us = (int32_t)((int64_t)med_adj * 1000000 / sample_rate);
                } else {
                    s_hub_splice_med_us = 0;
                }

                if (adj > 0) {
                    /* Its own buffer, not chunk: chunk holds the audio read at
                     * the top of this pass and not yet written to the DAC, so
                     * discarding into it dropped that and played the tail of the
                     * skipped region instead. Counted correctly either way, so
                     * nothing drifted -- it just played 5.8 ms of the wrong
                     * audio at every boundary. Same fix on the satellite. */
                    static uint8_t discard[AUDIO_CHUNK_BYTES];
                    int32_t left = adj;
                    while (left > 0) {
                        size_t want = (size_t)(left > AUDIO_FRAMES ? AUDIO_FRAMES : left)
                                      * AUDIO_CHANNELS * sizeof(int16_t);
                        size_t g = xStreamBufferReceive(local_ring, discard, want, 0);
                        if (g == 0) break;
                        left -= g / (AUDIO_CHANNELS * sizeof(int16_t));
                    }
                    samples_played += (adj - left);
                    applied = adj - left;
                    ESP_LOGW(TAG, "track boundary: skipped %ld ms to null phase",
                             (long)(applied * 1000 / (int32_t)sample_rate));
                } else if (adj < 0) {
                    static const uint8_t quiet[AUDIO_CHUNK_BYTES] = {0};
                    int32_t left = -adj;
                    size_t w = 0;
                    while (left > 0) {
                        int32_t n = left > AUDIO_FRAMES ? AUDIO_FRAMES : left;
                        size_t bytes = (size_t)n * AUDIO_CHANNELS * sizeof(int16_t);
                        i2s_channel_write(i2s_tx, quiet, bytes, &w, portMAX_DELAY);
                        left -= n;
                    }
                    applied = adj;
                    ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                             (long)(-applied * 1000 / (int32_t)sample_rate));
                }
                if (adj != 0) {
                    n_splices++;
                    s_phase_stepped = true;   /* the average before it is stale */
                    /* And so is the splice's own history, for the same reason:
                     * every reading in it was taken before this unit moved. */
                    sync_phase_reset(&s_phase_hist);
                }

                /*
                 * One line per track for how far apart the speakers had drifted
                 * by the end of it -- the figure to compare sessions and builds
                 * on, because it is taken at the same point of every track
                 * cycle rather than wherever a log window happened to fall.
                 *
                 * The satellite figure is the marker: a physical measurement of
                 * when a sample reached the output, so it sees things no
                 * software reading can. The hub's splice is how much of its own
                 * error it had accumulated. They answer different questions and
                 * both belong here.
                 */
                s_hub_splice_us = (int32_t)((int64_t)applied * 1000000 / sample_rate);
                s_hub_splice_at = esp_timer_get_time();

                if (s_sync_at) {
                    ESP_LOGW(TAG, "TRACK DIVERGENCE: satellite %+lld us "
                                  "(marker, %lld ms before this boundary) | "
                                  "hub spliced %+ld ms | hub phase %+ld us "
                                  "(median %+ld us, would have spliced %+ld ms)",
                             s_sync_err_us,
                             (s_hub_splice_at - s_sync_at) / 1000,
                             (long)(s_hub_splice_us / 1000),
                             (long)s_phase_err_us,
                             (long)med_us, (long)(s_hub_splice_med_us / 1000));
                } else {
                    /* No marker wire -- the normal deployed case, since it is a
                     * bench instrument. Satellites report over WiFi instead, and
                     * their line arrives within PROBE_PERIOD_MS of this one. */
                    ESP_LOGW(TAG, "TRACK BOUNDARY: hub spliced %+ld ms | "
                                  "hub phase %+ld us (median %+ld us, would have "
                                  "spliced %+ld ms) | no marker fitted",
                             (long)(s_hub_splice_us / 1000),
                             (long)s_phase_err_us,
                             (long)med_us, (long)(s_hub_splice_med_us / 1000));
                }
                /*
                 * The visualiser is not told. A splice moves audio WITHIN the
                 * timeline to correct this unit's position in it; the timeline
                 * every frame is dated against and drawn on does not move, and
                 * arrival is untouched.
                 */
            }

            int32_t mark = s_marker_sample;
            if (mark >= 0 && samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
                /* 200 us of busy-wait in the playback path -- see the Kconfig
                 * help. Nothing corrects on what it measures. */
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
                s_marker_at = esp_timer_get_time();
                esp_rom_delay_us(MARKER_PULSE_US);
                gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
                s_marker_sample = -1;
            }
            samples_played += AUDIO_FRAMES;

            /* Last thing before the DMA buffer, and deliberately after every
             * count above: it rewrites slots within frames that already exist,
             * so samples_played and the phase queue are looking at the same
             * timeline whatever this unit's speaker is placed as. */
            audio_apply_channel_mode((int16_t *)chunk, AUDIO_FRAMES);

            size_t written = 0;
            const int64_t w0 = s_refill_active ? esp_timer_get_time() : 0;
            if (i2s_channel_write(i2s_tx, chunk, sizeof(chunk), &written,
                                  portMAX_DELAY) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            if (s_refill_active) {
                if (esp_timer_get_time() - w0 < REFILL_FAST_US) {
                    s_refill_frames += (int32_t)(written / (AUDIO_CHANNELS * sizeof(int16_t)));
                } else {
                    s_refill_active = false;
                    ESP_LOGW(TAG, "REFILL after %s: %ld frames (%ld ms) before a "
                                  "write blocked -- phase readings inside this "
                                  "window are not DAC-paced",
                             s_refill_why, (long)s_refill_frames,
                             (long)(s_refill_frames * 1000 / (int32_t)sample_rate));
                }
            }
            /* Immediately, and before anything else can delay this task: it is
             * the instant the next pass dates its phase reading from. */
            wrote_at = esp_timer_get_time();
        }
    }
}

/* ------------------------------------------------------- time probe server */

static void probe_task(void *arg)
{
    (void)arg;
    /* Holds a max log_msg_t (~222 bytes); the time/splice messages it also
     * fields are far smaller. */
    uint8_t buf[256];
    struct sockaddr_in from;

    while (1) {
        /* recvfrom writes the actual address length back, so this must be reset
         * every iteration rather than hoisted out of the loop. */
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        int64_t t2 = esp_timer_get_time();          /* stamp on arrival */

#if CONFIG_DANCEFLOOR_WIFI_LOGS
        /*
         * Collector registration. Not client_seen(): the laptop never sends a
         * time probe, so it must not enter s_clients, or it would be unicasted
         * audio like a satellite. Refreshed on every SUB; wifi_log ages the
         * address out after ~30 s if the collector stops sending them.
         */
        if (n >= (int)sizeof(log_sub_msg_t) && buf[0] == MSG_LOG_SUB) {
            log_sub_msg_t sub;
            memcpy(&sub, buf, sizeof sub);
            if (sub.magic == LOG_SUB_MAGIC) {
                wifi_log_note_collector(from.sin_addr.s_addr);
            }
            continue;
        }

        /*
         * A satellite's log line or HEALTH snapshot, relayed to the collector.
         * Stamped with the source address -- the satellite does not know its own
         * DHCP lease -- and forwarded verbatim. Not client_seen(): this is not a
         * probe, and the satellite that sent it is already registered by its
         * probes anyway. The whole relay is a couple of non-blocking sendto()s;
         * probe latency is unaffected.
         */
        if (buf[0] == MSG_LOG && n >= (int)(sizeof(log_msg_t) - LOG_MSG_MAX)) {
            log_msg_t *m = (log_msg_t *)buf;
            if (m->msg_len <= LOG_MSG_MAX &&
                n >= (int)LOG_MSG_BYTES(m->msg_len)) {
                m->src_ip = from.sin_addr.s_addr;
                wifi_log_send_to_dest(m, LOG_MSG_BYTES(m->msg_len));
            }
            continue;
        }
        if (buf[0] == MSG_HEALTH && n >= (int)sizeof(health_msg_t)) {
            health_msg_t *m = (health_msg_t *)buf;
            m->src_ip = from.sin_addr.s_addr;
            wifi_log_send_to_dest(m, sizeof *m);
            continue;
        }
#endif

        /*
         * A satellite reporting what it corrected at a track boundary. Both
         * units splice by their own error against the same published timeline,
         * so the DIFFERENCE between the two corrections is how far apart they
         * had drifted over that track -- the same question the marker GPIO
         * answers physically, over the WiFi that is there anyway.
         */
        if (n >= (int)sizeof(splice_msg_t) && buf[0] == MSG_SPLICE) {
            splice_msg_t s;
            memcpy(&s, buf, sizeof(s));
            client_seen(&from);
            const int64_t age = s_hub_splice_at ? (t2 - s_hub_splice_at) / 1000 : -1;
            const char *who = inet_ntoa(from.sin_addr);
            if (age >= 0 && age < 10000) {
                /*
                 * The second clause is the counterfactual, and it is the whole
                 * point of this build: what `apart` would have been had both
                 * units spliced on a median of their last few phase readings
                 * rather than on the newest one. Both figures are measured at
                 * the same boundary from the same histories, so they can be
                 * compared directly -- unlike two builds' log windows, which
                 * cannot, and which is how three wrong diagnoses were reached
                 * here. If the median column is consistently the smaller one
                 * across a session, the splice moves to it.
                 */
                ESP_LOGW(TAG, "TRACK DIVERGENCE (wifi): %s spliced %+ld ms "
                              "(phase %+ld us), hub spliced %+ld ms -> %+ld ms apart"
                              " | median: sat %+ld ms, hub %+ld ms -> %+ld ms apart",
                         who, (long)(s.applied_us / 1000), (long)s.phase_us,
                         (long)(s_hub_splice_us / 1000),
                         (long)((s.applied_us - s_hub_splice_us) / 1000),
                         (long)(s.applied_med_us / 1000),
                         (long)(s_hub_splice_med_us / 1000),
                         (long)((s.applied_med_us - s_hub_splice_med_us) / 1000));
            } else {
                /* No boundary of our own to compare against -- the hub's phase
                 * was invalid, or this arrived nowhere near one. */
                ESP_LOGW(TAG, "satellite %s spliced %+ld ms (phase %+ld us), "
                              "no hub boundary to compare",
                         who, (long)(s.applied_us / 1000), (long)s.phase_us);
            }
            continue;
        }

        if (n < (int)sizeof(time_msg_t) || buf[0] != MSG_TIME_REQ) {
            continue;
        }
        client_seen(&from);      /* probing implies listening */

        time_msg_t msg;
        memcpy(&msg, buf, sizeof(msg));
        msg.type = MSG_TIME_RSP;
        msg.t2 = t2;
        msg.t3 = esp_timer_get_time();              /* stamp immediately before send */
        sendto(sock, &msg, sizeof(msg), 0, (struct sockaddr *)&from, from_len);

        /*
         * Measurement only -- see tsf_msg_t. Sent on the back of the probe
         * reply because that already runs once per satellite per probe and
         * needs no task, no client snapshot and no timer of its own. 17 bytes
         * four times a second against ~42 kB/s of audio.
         *
         * The pair is read as close together as it can be: the gap between them
         * is skew that lands directly in the comparison.
         */
        const int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_AP);
        const int64_t now = esp_timer_get_time();
        /* Say which way it went, once, either way. Silence here would leave a
         * run with no TSF output ambiguous between "not supported" and "this
         * board is not running the branch". */
        static bool told;
        if (!told) {
            told = true;
            if (tsf == 0) {
                /* A persistent zero here IS the result: SoftAP-side TSF is not
                 * exposed on this target and the experiment stops. */
                ESP_LOGW(TAG, "TSF reads 0 on the AP interface -- nothing to compare");
            } else {
                ESP_LOGW(TAG, "TSF on the AP interface reads %lld us, sending to "
                              "satellites", tsf);
            }
        }
        if (tsf == 0) {
            continue;
        }
        tsf_msg_t tm = { .type = MSG_TSF, .tsf = tsf, .local = now };
        sendto(sock, &tm, sizeof(tm), 0, (struct sockaddr *)&from, from_len);
    }
}

/*
 * Servo the DAC clock on the buffer level, not on the measured rate.
 *
 * Chasing the measured rate cannot work: it carries ~0.3% noise, and whatever
 * error is left integrates straight into this buffer until it overflows or
 * empties. The level itself IS that integral, so nulling it removes the
 * accumulated error rather than the instantaneous one. Correction is spread over
 * ~40 s, well below the ~1% pitch shift a listener would notice.
 */
/*
 * Records only, and lives in IRAM. Both are load-bearing.
 *
 * IDF marks heap_caps_alloc_failed() HEAP_IRAM_ATTR precisely because the heap
 * is usable with the flash cache disabled, so this can be reached from an ISR
 * or from a task running while a flash write is in progress. A hook in flash
 * would fault there -- a diagnostic for running out of memory that crashes
 * under the one condition it exists to observe. Hence IRAM_ATTR, no string
 * literals, and nothing called that is not itself in IRAM.
 *
 * And no logging even where it would be reachable: this runs inside the
 * allocator, and ESP_LOGx allocates. ring_monitor_task says it within 5 s from
 * a context where saying things is safe.
 */
static IRAM_ATTR void on_alloc_failed(size_t size, uint32_t caps, const char *function_name)
{
    (void)function_name;
    n_alloc_fail++;
    if ((uint32_t)size > alloc_fail_size) {
        alloc_fail_size = (uint32_t)size;
        alloc_fail_caps = caps;
    }
}

static void ring_monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        const uint32_t heap_now = esp_get_free_heap_size();
        if (heap_now < heap_min_window) {
            heap_min_window = heap_now;
        }

        /* Said once, and within 5 s of the fact rather than at the next soak
         * line -- an allocation failure is the thing you want to see next to
         * whatever else the console was saying at the time. The running count
         * stays on HEALTH. */
        static uint32_t alloc_fail_told;
        if (n_alloc_fail != alloc_fail_told) {
            alloc_fail_told = n_alloc_fail;
            ESP_LOGE(TAG, "ALLOCATION FAILED %" PRIu32 " time(s): largest request %"
                          PRIu32 " B (caps 0x%" PRIx32 "), heap %" PRIu32
                          " free, largest block %u",
                     n_alloc_fail, alloc_fail_size, alloc_fail_caps, heap_now,
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
        }

        /*
         * Soak line, every 60 s, and deliberately ahead of the streaming check
         * below: if audio has stopped, that is exactly when the heap and the
         * counters matter most.
         *
         * Totals rather than rates. Everything else here is cleared every
         * window, which answers "what is happening now" and cannot answer "has
         * this been happening slowly for an hour" -- and the longest run this
         * system had ever been given was seven minutes.
         */
        static int health_left;
        if (--health_left <= 0) {
            health_left = 12;                      /* 12 x 5 s */
            hw_mon = uxTaskGetStackHighWaterMark(NULL);
            /* `window` is the lowest this minute, `min` the lowest since boot.
             * The pair is the point: a window far below the current free figure
             * dates the dip to this line, which the watermark alone never
             * could. Taken and cleared, like every other windowed counter. */
            const uint32_t heap_win = heap_min_window;
            heap_min_window = UINT32_MAX;
            ESP_LOGW(TAG, "HEALTH: up %llu s | heap %" PRIu32 " (min %" PRIu32
                          ", window %" PRIu32 ", largest %u) | "
                          "stack play %" PRIu32 " mon %" PRIu32 " | underruns %" PRIu32
                          " restarts %" PRIu32 " splices %" PRIu32 " retunes %" PRIu32
                          " (%" PRIu32 " refused) | sta-left %" PRIu32
                          " (dropped %" PRIu32 ", no-lease %" PRIu32
                          ") | sta-timeout %" PRIu32
                          " | alloc-fail %" PRIu32
                          " | phase-drop %" PRIu32 " short-reads %" PRIu32
                          " (%" PRIu32 " frames) | wifi-over %" PRIu32,
                     (unsigned long long)(esp_timer_get_time() / 1000000),
                     esp_get_free_heap_size(), esp_get_minimum_free_heap_size(),
                     heap_win == UINT32_MAX ? 0 : heap_win,
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
                     hw_play, hw_mon, n_underruns, n_restarts, n_splices,
                     n_retunes, n_retunes_bad, n_sta_left, n_sta_dropped,
                     n_sta_nolease, n_sta_timeout, n_alloc_fail,
                     n_phase_drop, n_short_reads, n_short_frames, n_wifi_oversize);

#if CONFIG_DANCEFLOOR_WIFI_LOGS
            /* The structured twin of the line above, for the collector's CSV.
             * The role-aliased fields carry this unit's counters: reanchors_or_
             * restarts = restarts, gaps_or_sta_left = sta-left, wifi_drops_or_
             * oversize = wifi-over, and the tail three are sta-dropped /
             * sta-nolease / sta-timeout (see health_msg_t). */
            static uint32_t health_seq;
            health_msg_t h;
            memset(&h, 0, sizeof h);
            h.type = MSG_HEALTH;
            h.role = LOG_ROLE_HUB;
            h.seq = health_seq++;
            h.uptime_s = (uint64_t)(esp_timer_get_time() / 1000000);
            h.heap_cur = esp_get_free_heap_size();
            h.heap_min = esp_get_minimum_free_heap_size();
            h.heap_win = heap_win == UINT32_MAX ? 0 : heap_win;
            h.heap_largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
            h.hw_play = hw_play;
            h.hw_mon = hw_mon;
            h.underruns = n_underruns;
            h.reanchors_or_restarts = n_restarts;
            h.splices = n_splices;
            h.retunes = n_retunes;
            h.retunes_refused = n_retunes_bad;
            h.gaps_or_sta_left = n_sta_left;
            h.wifi_drops_or_oversize = n_wifi_oversize;
            h.alloc_fail = n_alloc_fail;
            h.phase_drop = n_phase_drop;
            h.short_reads = n_short_reads;
            h.short_frames = n_short_frames;
            h.ring_full_or_sta_dropped = n_sta_dropped;
            h.upgrades_or_sta_nolease = n_sta_nolease;
            h.anchors_refused_or_timeout = n_sta_timeout;
            h.log_dropped = wifi_log_dropped();
            h.log_no_dest = wifi_log_no_dest();
            wifi_log_send_to_dest(&h, sizeof h);
#endif
        }

        if (local_start == 0 || rate_ema == 0) {
            continue;
        }

#if CONFIG_DANCEFLOOR_RETUNE_BENCH_S > 0
        /* Bench: retune to the rate already set, so the RETUNE COST line below
         * reports the cost of retuning and nothing else. One unit at a time. */
        static int bench_left;
        if (--bench_left <= 0) {
            bench_left = (CONFIG_DANCEFLOOR_RETUNE_BENCH_S + 4) / 5;
            ESP_LOGW(TAG, "BENCH: forcing a same-rate retune at %" PRIu32 " Hz", tx_rate);
            retune_dac(tx_rate);
            continue;
        }
#endif

        /*
         * The servo input, matching the satellite: a 4-sample EMA of the phase,
         * forgotten after a splice because the average from before it describes
         * a situation that no longer exists.
         *
         * This unit used to act on the raw reading, and the raw reading is far
         * noisier than anyone had established. Measured here, two reads of
         * s_phase_err_us one millisecond apart:
         *
         *   local ring ... | phase +26786 us (smoothed +8996 us)
         *   servo: phase +11108 us (smoothed +8996), ...
         *
         * 15.7 ms of swing between consecutive samples, with the average
         * sitting still at +9 ms through it. That is the "hub absolute phase
         * does not settle" wart in docs/clock-sync.md, quantified: the servo was
         * substantially triggering on measurement noise. A shadow run put two
         * of six retunes at the deadband edge, both of which the average would
         * have held.
         *
         * Honest caveat: cross-unit audio measured 0.5 to 2.5 ms with the raw
         * input, which is already the best this project has recorded, so this
         * is expected to reduce pointless retunes rather than to move that
         * number. If it moves it the wrong way, revert this commit -- the raw
         * value is still computed below and still logged.
         */
        static int32_t s_err_ema;
        static bool    s_err_ema_valid;
        if (s_phase_stepped || !s_phase_valid) {
            s_phase_stepped = false;
            s_err_ema_valid = false;   /* history describes a different world */
        }
        s_err_ema = s_err_ema_valid ? (s_err_ema * 3 + s_phase_err_us) / 4
                                    : s_phase_err_us;
        s_err_ema_valid = true;

        size_t filled = LOCAL_RING_BYTES - xStreamBufferSpacesAvailable(local_ring);
        /* Printed once per log period, not every window. tx-fail accumulates
         * across the quiet windows so nothing is lost by not printing it. */
        static int status_left;
        if (--status_left <= 0) {
            status_left = CONFIG_DANCEFLOOR_LOG_PERIOD_S / 5;
            ESP_LOGI(TAG, "local ring %u bytes (%lu ms) | phase %+ld us (smoothed %+ld us) | "
                          "tx-fail %" PRIu32,
                     (unsigned)filled,
                     (unsigned long)(filled * 1000 / (sample_rate * AUDIO_CHANNELS * 2)),
                     (long)s_phase_err_us, (long)s_err_ema, s_tx_fail);
            s_tx_fail = 0;
        }

        const int32_t target = (int32_t)(LEAD_US / 1000) *
                               (int32_t)(rate_ema * AUDIO_CHANNELS * 2 / 1000);
        int32_t err_frames = ((int32_t)filled - target) / (AUDIO_CHANNELS * 2);
        int32_t depth_ms = err_frames * 1000 / (int32_t)rate_ema;

        if (!s_phase_valid) {
            continue;
        }
        /*
         * Phase drives the correction; buffer depth is only a guard against
         * running empty or overflowing, which phase control would not see
         * coming. Late means behind the timeline, so play faster.
         *
         * Spread over ~100 s: at 40 s the loop was still correcting after the
         * error had gone and overshot to +8 ms. Real drift is ~0.8 ms/minute,
         * far slower than the correction needs to be.
         */
        int32_t adj = (int32_t)((int64_t)s_err_ema * rate_ema / 100000000LL);
        /* The drift correction is small by nature -- real drift is ~14 ppm.
         * Anything larger is a bad phase reading, not a rate error. */
        if (adj >  RATE_TRIM_MAX_HZ) adj =  RATE_TRIM_MAX_HZ;
        if (adj < -RATE_TRIM_MAX_HZ) adj = -RATE_TRIM_MAX_HZ;
        if (depth_ms < -120) {
            adj = -20;
        } else if (depth_ms > 120) {
            adj = 20;
        }
        uint32_t desired = (uint32_t)((int32_t)rate_ema + adj);

        /*
         * Deadband in phase error, not in rate -- see PHASE_DEADBAND_US. The
         * old tx_rate/5000 was documented as ~8 ms and is really ~20 ms. This
         * unit has always parked its playback across a retune, so its retunes
         * were never the expensive kind; the satellite's were, until it got the
         * same guard.
         */
        int32_t deadband = (int32_t)((int64_t)PHASE_DEADBAND_US * rate_ema / 100000000LL);
        if (deadband < 1) {
            deadband = 1;
        }

        /* Wait for the buffer to respond before correcting again -- the hub
         * had no cooldown at all, so it retuned every window and chased its own
         * previous correction. */
        /* The raw input is the shadow now. Kept so the comparison survives the
         * change, and so a revert has something to check itself against. */
        int32_t adj_raw = (int32_t)((int64_t)s_phase_err_us * rate_ema / 100000000LL);
        if (adj_raw >  RATE_TRIM_MAX_HZ) adj_raw =  RATE_TRIM_MAX_HZ;
        if (adj_raw < -RATE_TRIM_MAX_HZ) adj_raw = -RATE_TRIM_MAX_HZ;
        const uint32_t desired_raw = (uint32_t)((int32_t)rate_ema + adj_raw);

        static int cooldown;
        if (cooldown > 0) {
            cooldown--;
        } else {
            const bool ema_would = desired     > tx_rate + (uint32_t)deadband ||
                                   desired     < tx_rate - (uint32_t)deadband;
            const bool raw_would = desired_raw > tx_rate + (uint32_t)deadband ||
                                   desired_raw < tx_rate - (uint32_t)deadband;
            /*
             * Still logged, with the roles swapped: each of these is now a
             * retune the raw input would have made and the average declined, or
             * the reverse. If these become common AND the cross-unit figure
             * degrades, this commit is the thing to revert.
             */
            if (raw_would != ema_would) {
                ESP_LOGW(TAG, "SERVO DIVERGES: smoothed %+ld us -> %" PRIu32 " Hz (%s), "
                              "raw %+ld us -> %" PRIu32 " Hz (%s)",
                         (long)s_err_ema,      desired,     ema_would ? "retune" : "hold",
                         (long)s_phase_err_us, desired_raw, raw_would ? "retune" : "hold");
            }
            if (ema_would) {
                ESP_LOGI(TAG, "servo: smoothed %+ld us (raw %+ld), buffer %+ld ms "
                              "-> DAC %" PRIu32 " Hz",
                         (long)s_err_ema, (long)s_phase_err_us, (long)depth_ms, desired);
                retune_dac(desired);
                cooldown = 4;          /* ~20 s against a 100 s correction */
            }
        }
    }
}

void streamer_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Before anything else allocates, so a failure during WiFi or socket setup
     * is caught too -- that is the phase with the largest single requests. */
    ESP_ERROR_CHECK(heap_caps_register_failed_alloc_callback(on_alloc_failed));

    local_ring = xStreamBufferCreate(LOCAL_RING_BYTES, AUDIO_CHUNK_BYTES);
    assert(local_ring);

    wifi_start_ap();
    socket_start();
    /* Mirror this hub's own logs and relay every satellite's to a collector.
     * Destination starts unset -- the collector announces itself with
     * MSG_LOG_SUB (handled in probe_task) and wifi_log_note_collector() points
     * this at it. No-op unless CONFIG_DANCEFLOOR_WIFI_LOGS is set. */
    wifi_log_init(LOG_ROLE_HUB, NULL);
    i2s_start(sample_rate);
#if CONFIG_DANCEFLOOR_ENABLE_MARKER
    marker_start();
#endif

    ESP_LOGI(TAG, "free heap after WiFi init: %" PRIu32 " bytes", esp_get_free_heap_size());

    xTaskCreate(probe_task, "probe", 4096, NULL, 6, NULL);
    xTaskCreatePinnedToCore(local_play_task, "play", 4096, NULL, 8, NULL, 1);
    xTaskCreate(ring_monitor_task, "ringmon", 3072, NULL, 3, NULL);
#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER
    /*
     * Safe before visualiser_start(): this only stores a pointer the analysis
     * task reads, and that task does not exist yet.
     *
     * Registered only when DANCEFLOOR_PUBLISH_FRAMES is on. The hub's own strip
     * renders from the analysis either way; this gates only whether those frames
     * go out to satellites. A floor where every satellite is LED_SOURCE_LOCAL
     * analyses its own audio and gains nothing from the ~5 kB/s per listener this
     * costs -- turn it off there to recover the airtime at scale. The per-satellite
     * case (some of each) wants a subscribe message, not a build switch.
     */
#if CONFIG_DANCEFLOOR_PUBLISH_FRAMES
    visualiser_set_publish(publish_frame);
#endif
#if CONFIG_DANCEFLOOR_PUBLISH_ML
    visualiser_set_ml_publish(publish_ml);
#endif
#endif
    ESP_LOGI(TAG, "streaming on port %d, unicast to registered listeners", SYNC_PORT);
}
