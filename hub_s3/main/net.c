/*
 * Bringing up the radio and the socket everything else talks over.
 *
 * The SoftAP configuration is mostly a list of things that were measured going
 * wrong: HT20 because the driver negotiates HT40 and nothing here can use it,
 * PMF off because its teardown was disassociating our own satellite, power save
 * off because it parks the radio between beacons.
 */
#include "hub.h"

#include <math.h>
#include <stdlib.h>

#include "esp_private/wifi.h"
#include "nvs_flash.h"

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
        n_join_churn++;
        const wifi_event_ap_stadisconnected_t *ev = data;
        if (ev) {
            client_gone(ev->mac);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_ASSIGNED_IP_TO_CLIENT) {
        n_join_churn++;
        /* Carries the address AND the MAC outright, so unlike the departure
         * above this needs no lease lookup and cannot fail to identify the
         * station -- which is what lets client_joined() seed ARP. */
        const ip_event_assigned_ip_to_client_t *ev = data;
        if (ev) {
            client_joined(ev->mac, &ev->ip);
        }
    }
}

#if CONFIG_DANCEFLOOR_WIFI_CHANNEL == 0
/*
 * Which channel to be an AP on, decided once, at boot.
 *
 * WHY NOT A CONSTANT. The channel has to be fixed for the session -- Bluetooth's
 * AFH routes around a known interferer far better than a moving one, and
 * bt_bridge carries the A2DP source over the same air -- but nothing in that
 * argument says it has to be fixed at COMPILE time, and this floor changes
 * venue. The 11 that used to be the default was the Kconfig default, never a
 * measurement.
 *
 * ALL THIRTEEN CHANNELS ARE SCANNED, not just the three candidates, and the
 * survey that prompted this is why. In one workshop channels 1 and 6 each
 * looked nearly bare on their own centre; four networks sitting on channel 5
 * were the largest contributor to both, and summed occupancy came out 533 and
 * 608 against channel 11's 87. A scan restricted to 1/6/11 would have read 1
 * and 6 as clear and picked one of them.
 *
 * SUMMED IN LINEAR POWER, because adding dBm figures is meaningless. The total
 * is converted back to dBm only to print it, so the log line reads in the same
 * unit as every other signal figure here.
 *
 * WHAT THIS DOES NOT MEASURE: airtime. A beacon says a network is present, not
 * that it is busy, and three idle neighbours can cost less than one saturated
 * one. This is the best guess available in about 1.6 s of boot, and what it
 * really buys is never sitting on a channel nobody looked at.
 */

/* 2.4 GHz channels sit 5 MHz apart and are 22 MHz wide, so anything within
 * four of a candidate lands on top of it. */
#define CHANNEL_OVERLAP 4

/* Where the survey lands if it cannot run at all. One of the non-overlapping
 * three, and 11 because that is what this project ran on for its whole
 * recorded history -- a fallback boot then behaves like every log already in
 * tools/soak rather than like nothing that came before. */
#define CHANNEL_FALLBACK 11

/* A linear power sum back to dBm for printing. An untouched channel reports a
 * floor rather than the -inf that log10(0) would give. */
static int occupancy_dbm(float mw)
{
    return mw > 0.0f ? (int)lroundf(10.0f * log10f(mw)) : -100;
}

static int survey_channel(void)
{
    static const int cand[3] = {1, 6, 11};

    /* Scanning needs STA mode and a started radio. No STA netif is created for
     * it: a scan wants no IP stack, and the AP netif made above is untouched. */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* PASSIVE. This unit has no business transmitting probe requests into a
     * room it is about to be the AP of, and a beacon interval is ~102 ms, so
     * 120 ms a channel catches at least one from everything that is there.
     * Thirteen channels at 120 ms is ~1.6 s, spent once. */
    const wifi_scan_config_t sc = {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_PASSIVE,
        .scan_time = { .passive = 120 },
    };
    esp_err_t err = esp_wifi_scan_start(&sc, true);

    float power[3] = {0};
    int nets[3] = {0};
    int seen = 0;

    if (err == ESP_OK) {
        /* One record at a time, which frees each as it goes -- the bulk call
         * would want an array sized for a band this unit has not seen yet, and
         * internal RAM here runs at ~12 kB free. Draining to ESP_FAIL is also
         * what releases the list, so there is nothing left to clear. */
        wifi_ap_record_t rec;
        while (esp_wifi_scan_get_ap_record(&rec) == ESP_OK) {
            seen++;
            for (int k = 0; k < 3; k++) {
                if (abs((int)rec.primary - cand[k]) <= CHANNEL_OVERLAP) {
                    power[k] += powf(10.0f, rec.rssi / 10.0f);
                    nets[k]++;
                }
            }
        }
    }

    ESP_ERROR_CHECK(esp_wifi_stop());

    if (err != ESP_OK) {
        /* Loud, because the quiet version of this is a board sitting on an
         * unmeasured channel while the log implies one was chosen. */
        ESP_LOGE(TAG, "channel survey FAILED (%s) -- using ch %d unmeasured; "
                      "the band was never looked at",
                 esp_err_to_name(err), CHANNEL_FALLBACK);
        return CHANNEL_FALLBACK;
    }

    int best = 0;
    for (int k = 1; k < 3; k++) {
        if (power[k] < power[best]) {
            best = k;
        }
    }

    /* `key value` pairs so tools/soak/capture.py lands every figure in
     * metrics.csv without a parser change, and a soak records the band it ran
     * in rather than only the channel it picked. */
    ESP_LOGW(TAG, "channel survey: nets-seen %d | ch1-dbm %d ch1-nets %d | "
                  "ch6-dbm %d ch6-nets %d | ch11-dbm %d ch11-nets %d | chose %d",
             seen,
             occupancy_dbm(power[0]), nets[0],
             occupancy_dbm(power[1]), nets[1],
             occupancy_dbm(power[2]), nets[2],
             cand[best]);
    return cand[best];
}
#endif /* CONFIG_DANCEFLOOR_WIFI_CHANNEL == 0 */

void wifi_start_ap(void)
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

    /* Before the AP config below, because this is what wc.ap.channel gets, and
     * after esp_wifi_init() because the scan needs a driver. */
#if CONFIG_DANCEFLOOR_WIFI_CHANNEL == 0
    const int channel = survey_channel();
#else
    const int channel = CONFIG_DANCEFLOOR_WIFI_CHANNEL;
    ESP_LOGW(TAG, "channel pinned to %d, no survey -- runs that must compare "
                  "with each other want this", channel);
#endif

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
    /* One of the three limits that have to agree; see MAX_CLIENTS, which is the
     * same number and carries the reasoning. The driver's own ceiling is
     * ESP_WIFI_MAX_CONN_NUM. */
    wc.ap.max_connection = MAX_CLIENTS;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wc.ap.channel = channel;
    wc.ap.dtim_period = 1;
    /*
     * 100 TU, which is IDF's floor, IDF's default, and -- measured -- the only
     * value this field has ever actually held on this hub.
     *
     * A SoftAP cannot send group-addressed frames whenever it likes -- stations
     * may be asleep -- so it buffers them and releases them after each DTIM
     * beacon. dtim_period is already 1, so that is every beacon, and the
     * interval is what is left. At 100 TU (1 TU = 1024 us) the hold is 102.4 ms
     * and every audio chunk, analysis frame and level occupies one static TX
     * buffer until its window opens.
     *
     * THIS LINE USED TO READ 50, AND THE RADIO NEVER TOOK IT. The field is
     * specified in TU, "multiples of 100, range 100 ~ 60000" -- see
     * wifi_ap_config_t in esp_wifi_types_generic.h -- and the comment that used
     * to sit here reasoned about it in MILLISECONDS, halving 100 to 50 to halve
     * the average occupancy. esp_wifi_set_config() returned ESP_OK, which says
     * only that IDF does not validate the field on the way in. The read-back
     * added below reports what the driver kept:
     *
     *     AP beacon_interval 100 TU (asked 50), dtim_period 1 (asked 1)
     *
     * So the hold was 102.4 ms the whole time, the "halving" never happened,
     * and any argument resting on it was resting on nothing. Set to 100 now so
     * the source says what the radio does.
     *
     * AND IT CANNOT BE SHORTENED. 100 TU is the bottom of the documented range
     * and dtim_period is already at its minimum of 1, so 102.4 ms was a FIXED
     * cost that the group burst had to be designed against rather than tuned
     * away. TX_FRAME_PACE_US in hub.h is still that number, though nothing on
     * this build waits for a beacon any more -- every lane is unicast now. See
     * the note beside TX_FRAME_PACE_US for what the pace is still doing and
     * what has not been re-measured.
     *
     * WHY THE HOLD WAS REAL, kept because the correction was hard-won. Power
     * save is not a consideration here -- every satellite sets WIFI_PS_NONE --
     * and that does NOT mean group frames went out immediately: the AP buffers
     * them for DTIM regardless of whether any station is sleeping. The
     * 2026-08-20 soak measured it directly, with ENOMEM refusals arriving at
     * the beacon rate (median 40 bursts per 5 s window against 48.8 beacons).
     * clients.c claimed the opposite and was corrected.
     */
    wc.ap.beacon_interval = 100;
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
     * What the driver ACTUALLY HOLDS, read back rather than assumed.
     *
     * This line is why the block above now reads 100. It was added while that
     * one still said 50, and on its first boot it printed
     * `beacon_interval 100 TU (asked 50)` -- the driver had been quietly
     * discarding the value for as long as it had been written, because
     * ESP_ERROR_CHECK passing says only that IDF does not validate the field on
     * the way in, not that the driver kept it.
     *
     * KEPT NOW AS A REGRESSION GUARD, though a weaker one than it was. The
     * numbers it guards -- TX_FRAME_PACE_US in hub.h, the buffer count in
     * sdkconfig.defaults -- were derived when a 102.4 ms hold applied to the
     * audio and frame lanes, and both are unicast now, so a change in the
     * interval would no longer break them the way it once would. It stays
     * because the derivation is still what the source says, and one line at
     * boot is a cheap way to notice if the premise moves.
     *
     * A status field would be wrong: it cannot change while running.
     */
    wifi_config_t back = {0};
    if (esp_wifi_get_config(WIFI_IF_AP, &back) == ESP_OK) {
        ESP_LOGW(TAG, "AP beacon_interval %u TU (asked %u), dtim_period %u "
                      "(asked %u) -- any group frame is held for beacon x dtim",
                 (unsigned)back.ap.beacon_interval, (unsigned)wc.ap.beacon_interval,
                 (unsigned)back.ap.dtim_period, (unsigned)wc.ap.dtim_period);
    } else {
        ESP_LOGW(TAG, "AP config read-back failed -- beacon/DTIM hold unknown");
    }

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
     * The channel stays pinned for the session: it costs nothing and helps
     * Bluetooth's adaptive frequency hopping route around us. WHICH channel is
     * now chosen at boot rather than at compile time -- see survey_channel()
     * above; AFH needs the value known and stationary, not known early.
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
             AP_SSID, AP_PASS, channel);
}

/*
 * WHY a sendto() failed, not just how often.
 *
 * s_tx_fail has read 0 in every log this unit has ever produced, so until it
 * did not, the count alone was the whole instrument. A non-zero figure has at
 * least two completely different causes with completely different fixes, and
 * the count cannot tell them apart:
 *
 *   ENOMEM        the WiFi driver is out of TX buffers. A load or memory
 *                 problem -- internal RAM here runs at ~8 kB free with a
 *                 largest block of ~3.5 kB, and TX buffers must be internal.
 *   EHOSTUNREACH  lwIP has no ARP entry and the pending-ARP queue is dropping
 *                 its overflow. A JOIN problem, and precisely what the ARP
 *                 seeding in client_joined() exists to prevent -- see the note
 *                 there, and the 161 tx-fails e6f03d1 produced without it.
 *
 * The first is fixed by taking work or memory off this board; the second by
 * looking at why a station was registered without a seeded entry. Guessing
 * between them from a bare count is how an evening gets spent on the wrong one.
 *
 * A TALLY rather than a log line per failure: these arrive up to ~200 per 20 s
 * window, and one line each would push the console out of the way of everything
 * it is meant to be read beside.
 *
 * Raced, deliberately, exactly as the counter it sits beside already is: three
 * tasks send (timeline, and the frame and ML publishers) and none of them take
 * a lock to increment s_tx_fail. The cost of losing one is a diagnostic that
 * reads 189 instead of 190, and the cost of a critical section on the send path
 * is real. Same trade the counters in hub_state.c already make.
 */
#define TX_ERR_SLOTS 4
static struct {
    int err;
    uint32_t n;
} s_tx_err[TX_ERR_SLOTS];
static uint32_t s_tx_err_other;   /* more distinct codes than slots */

/*
 * THE SHAPE OF AN ENOMEM STORM, which is a different question from its size and
 * is the one the 2026-08-20 soak could not answer.
 *
 * That run refused 54% of the audio stream for its last eight minutes, and the
 * arithmetic says the FILL alone cannot do that: ~50 audio + ~27 frames + 1 vol
 * is ~78 group packets/s, which is ~4 per beacon interval against 38 static TX
 * buffers. For the pool to sit exhausted across twenty-five consecutive windows
 * something has to be wrong with the DRAIN -- and nothing on this board could
 * see the difference, because a count of refusals looks identical either way.
 *
 * The tell is PERIODICITY. A SoftAP releases group-addressed frames after the
 * DTIM beacon, so if the release is happening on schedule and the queue is
 * merely deeper than one release can clear, refusals arrive in clusters spaced
 * one beacon apart -- ~51 ms, or ~102 ms if the out-of-range beacon_interval in
 * wifi_start_ap() is being clamped to IDF's default. Read `burst-gap` against
 * those two numbers. If instead one burst runs continuously for hundreds of ms
 * with no spacing to speak of, the release itself has stalled and the frame
 * lane is not the culprit however much of the pool it holds.
 *
 * A stretch of refusals ENDS WHEN AN AUDIO SEND SUCCEEDS, which is the physical
 * event of interest -- a buffer came back -- rather than a timeout. A timeout
 * was tried first and cannot work here, which is worth writing down so it is
 * not tried again: audio offers 50 packets/s, so refusals are ~20 ms apart even
 * in a total stall, while consecutive beacon-released clusters are separated by
 * the beacon MINUS however long the cluster ran. There is no threshold that
 * splits the second case without also splitting the first. A host test of the
 * timeout version read four beacon-spaced clusters as two bursts.
 *
 * So: audio is the probe. It is the right one because it is the lane that is
 * never gated -- fan_out() ignores the backoff by design -- so it keeps asking
 * the pool at a steady 50/s throughout. The frame lane's successes would be a
 * poor probe because it stops asking the moment it is refused.
 *
 * Gauges, cleared by the window that prints them, and raced like everything
 * else on this path.
 */
static bool s_refusing;             /* a refusal stretch is open right now */
static int64_t s_refuse_since;      /* when it opened */
static int64_t s_prev_burst_at;     /* start of the previous stretch */
static uint32_t n_enomem_bursts;    /* stretches that OPENED in this window */
/*
 * Refusals in this window, and the only thing that licenses the line to print.
 *
 * Without it a window that merely INHERITED an open stretch, and then saw it
 * close with nothing refused, reported `enomem-bursts 1 | refuse-max 0 ms` --
 * a burst that did not happen, on the very window that says the hub recovered.
 * A host test caught it; the earlier hand-written version of that test did not,
 * because it carried its own copy of this code.
 */
static uint32_t n_enomem_refusals;
static bool s_carried_open;         /* a stretch was already open at window start */
static int32_t n_refuse_max_us;

/*
 * WHERE THE STRETCHES FALL RELATIVE TO THE BEACON, as a histogram rather than a
 * single number.
 *
 * This replaces a MINIMUM, which was the wrong statistic and nearly cost a
 * wrong conclusion. A minimum is set by the single tightest pair in the window,
 * so it read 4-30 ms through the 2026-08-20 join transient while the stretches
 * were in fact arriving at the beacon rate -- the beacon-lock finding had to be
 * argued from the stretch COUNT instead (median 40 per 5 s window against 48.8
 * beacons), which is indirect and only works while the count is high.
 *
 * Four fixed buckets around the 102.4 ms hold, so the shape is legible at a
 * glance without carrying an array of samples:
 *
 *   <25 ms     back-to-back: the pool is refusing continuously, not in clusters
 *   25-75      sub-beacon: something is releasing faster than DTIM, or the
 *              stretch is being broken by a unicast send getting through
 *   75-150     THE BEACON. A pile here is the DTIM hold, stated directly.
 *   >150       multi-beacon: a release came round and did not clear the backlog
 *
 * Read the buckets against each other, not the absolute counts: `gaps
 * 2/1/44/3` is a beacon-locked queue, `gaps 61/0/0/0` is a stall that never
 * lets go.
 */
#define BURST_GAP_BUCKETS 4
static uint32_t n_burst_gap[BURST_GAP_BUCKETS];

static void enomem_note_shape(int64_t now)
{
    n_enomem_refusals++;
    if (!s_refusing) {
        s_refusing = true;
        s_refuse_since = now;
        n_enomem_bursts++;
        if (s_prev_burst_at) {
            /* Start-to-start, not end-to-start: a beacon releases on a fixed
             * period, so it is the period between stretches that should land on
             * the beacon interval. An end-to-start gap would be that period
             * minus however long the stretch ran, which is the quantity that
             * made the timeout version unreadable. */
            const int64_t gap_us = now - s_prev_burst_at;
            const int32_t gap_ms = (int32_t)(gap_us / 1000);
            const int b = (gap_ms < 25)  ? 0 :
                          (gap_ms < 75)  ? 1 :
                          (gap_ms < 150) ? 2 : 3;
            n_burst_gap[b]++;
        }
        s_prev_burst_at = now;
    }
    const int64_t len = now - s_refuse_since;
    if (len > n_refuse_max_us && len < INT32_MAX) {
        n_refuse_max_us = (int32_t)len;
    }
}

/*
 * An audio packet reached the transmit path, so the pool has a buffer again.
 * Called from fan_out() on FANOUT_SENT -- see the note above for why audio and
 * not any other lane is what closes a stretch.
 */
void tx_send_ok(void)
{
    s_refusing = false;
}

/*
 * Render the burst shape and clear it. Empty while nothing has been refused, so
 * a clean window's line stays byte-for-byte what it was before this existed --
 * the same rule tx_fail_summary() follows and for the same reason.
 */
void tx_burst_summary(char *buf, size_t len)
{
    if (!n_enomem_refusals) {
        buf[0] = '\0';
        /* Nothing was refused, so nothing is claimed -- but whether a stretch
         * is open still has to reach the next window, or a stall that goes
         * quiet for one window and resumes would read as two. */
        s_carried_open = s_refusing;
        return;
    }
    /* A stretch that OPENED before this window still ran through it, and is one
     * of this window's bursts even though nothing here opened it. */
    const uint32_t bursts = n_enomem_bursts + (s_carried_open ? 1u : 0u);
    /*
     * The buckets always print, even all-zero, and the labels are in the name:
     * `gaps <25/25-75/75-150/>150`. A single stretch produces no gap at all --
     * a gap needs two -- and four zeros say that plainly, where the old single
     * figure had to print the word "none" to avoid being read as "no time
     * between clusters", which is the opposite of what it meant.
     */
    snprintf(buf, len, " | enomem-bursts %" PRIu32 " | refuse-max %ld ms"
                       " | gaps %" PRIu32 "/%" PRIu32 "/%" PRIu32 "/%" PRIu32
                       " | refuse-near-frame %" PRIu32
                       " | audio-retry %" PRIu32 " | audio-retry-ok %" PRIu32,
             bursts, (long)(n_refuse_max_us / 1000),
             n_burst_gap[0], n_burst_gap[1], n_burst_gap[2], n_burst_gap[3],
             n_refuse_near_frame, n_audio_retry, n_audio_retry_ok);
    /* Cleared here, with the burst shape they are read against: all of it
     * describes one window and a counter that outlived its window would be
     * attributed to the next one's refusals. */
    n_refuse_near_frame = 0;
    n_audio_retry = 0;
    n_audio_retry_ok = 0;

    /*
     * A STRETCH STILL OPEN AT THE BOUNDARY IS CARRIED, NOT DROPPED.
     *
     * Dropping it made a multi-window stall -- the single worst thing this
     * instrument exists to catch -- print once and then report nothing at all,
     * because no NEW stretch ever began. Found by a host test, and it would
     * have read as a hub that recovered.
     *
     * Carried as a FLAG rather than as a count, which was the second bug on the
     * same line: a count survives into a window that refuses nothing, and
     * prints a burst that did not happen. The flag is only ever honoured beside
     * a real refusal, above.
     *
     * s_refusing and s_refuse_since survive, so refuse-max keeps growing from
     * the true start and the next window says "this stall is now 3.4 s long"
     * rather than restarting the clock. s_prev_burst_at survives too: a gap is
     * a real interval whether or not a window boundary fell inside it.
     */
    s_carried_open = s_refusing;
    n_enomem_bursts = 0;
    n_enomem_refusals = 0;
    n_refuse_max_us = 0;
    memset(n_burst_gap, 0, sizeof(n_burst_gap));
}

/*
 * Render the lane breakdown and clear it. Always non-empty -- it is printed
 * inside the existing `(N audio)` parentheses, which have never been optional.
 */
void tx_fail_lanes(char *buf, size_t len)
{
    static const char *const name[TX_LANE_N] = {
        "audio", "frame", "vol", "meta", "probe",
    };
    size_t off = 0;
    buf[0] = '\0';
    for (int i = 0; i < TX_LANE_N; i++) {
        const uint32_t n = s_tx_lane_fail[i];
        s_tx_lane_fail[i] = 0;
        /* AUDIO always prints, even at zero: it is the number the room cares
         * about and its absence must not read as "not measured". The rest print
         * only when they have something to say, so a clean line stays short. */
        if (!n && i != TX_LANE_AUDIO) {
            continue;
        }
        off += snprintf(buf + off, off < len ? len - off : 0, "%s%" PRIu32 " %s",
                        off ? ", " : "", n, name[i]);
    }
}

/*
 * The audio downlink's own entry point.
 *
 * tx-fail was one number shared by audio, analysis frames, ML results, metadata
 * and the log shipper, so `tx-fail 92` could not say how many satellite gaps
 * this hub had caused itself -- and that is the only part of it that is
 * audible. A refused frame costs one repaint; a refused audio packet is a hole
 * in the sound on every satellite at once, and under FEC it is recoverable only
 * if the NEXT packet gets through, which under a burst of ENOMEM is exactly
 * what does not happen.
 *
 * Kept as a named wrapper now that tx_fail_note() takes a lane: the two audio
 * send paths are its only callers, they are three lines apart in timeline.c, and
 * three comments there refer to it by name.
 */
void tx_fail_note_audio(int err)
{
    tx_fail_note(TX_LANE_AUDIO, err);
}

void tx_fail_note(tx_lane_t lane, int err)
{
    s_tx_fail++;
    if ((unsigned)lane < TX_LANE_N) {
        s_tx_lane_fail[lane]++;
    }
    /*
     * ENOMEM is the WiFi pool out of TX buffers, and it is the one failure a
     * non-audio lane can do something about: back off for TX_BACKOFF_US so
     * publish_frame yields and the buffers audio is being refused are left for
     * audio. fan_out() does not check this, so audio keeps sending and,
     * if it still hits ENOMEM, re-arms the deadline from here. The errno tally
     * above stays the whole of the diagnosis; this is the reaction to it.
     */
    if (err == ENOMEM) {
        const int64_t now = esp_timer_get_time();
        enomem_note_shape(now);
        s_tx_congested_until = now + TX_BACKOFF_US;
        /* Only AUDIO asks the question, because only audio is the victim: a
         * frame refused while another frame is in flight is the lane queueing
         * behind itself, which says nothing about who starved the room. See
         * n_refuse_near_frame in hub.h for what the two readings mean. */
        if (lane == TX_LANE_AUDIO
            && now - s_tx_frame_sent_us <= TX_NEAR_US) {
            n_refuse_near_frame++;
        }
    }
    for (int i = 0; i < TX_ERR_SLOTS; i++) {
        if (s_tx_err[i].n == 0) {         /* free slot: claim it */
            s_tx_err[i].err = err;
            s_tx_err[i].n = 1;
            return;
        }
        if (s_tx_err[i].err == err) {
            s_tx_err[i].n++;
            return;
        }
    }
    s_tx_err_other++;
}

/*
 * Render the tally and clear it, for the status line in servo.c.
 *
 * Writes an EMPTY string when nothing failed, so a clean run's line is byte-for
 * byte what it has always been and stays comparable with every log captured
 * before this instrument existed. Cleared here because the caller zeroes
 * s_tx_fail on the same line; the two must not drift apart.
 */
void tx_fail_summary(char *buf, size_t len)
{
    size_t off = 0;
    buf[0] = '\0';
    for (int i = 0; i < TX_ERR_SLOTS; i++) {
        if (s_tx_err[i].n == 0) {
            continue;
        }
        off += snprintf(buf + off, off < len ? len - off : 0, "%s%s %" PRIu32,
                        off ? ", " : " -- ", strerror(s_tx_err[i].err),
                        s_tx_err[i].n);
        s_tx_err[i].n = 0;
    }
    if (s_tx_err_other) {
        snprintf(buf + off, off < len ? len - off : 0, "%sother %" PRIu32,
                 off ? ", " : " -- ", s_tx_err_other);
        s_tx_err_other = 0;
    }
}

void socket_start(void)
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
