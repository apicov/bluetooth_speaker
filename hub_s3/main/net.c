/**
 * @file net.c
 * @brief Bringing up the radio and the socket everything else talks over, and
 *        the transmit-path instruments the status line is built from.
 *
 * The SoftAP configuration is largely a list of things measured going wrong:
 * HT20 because the driver negotiates HT40 and nothing here can use it, PMF off
 * because its teardown was disassociating our own satellite, power save off
 * because it parks the radio between beacons. Which channel it comes up on is
 * surveyed at boot rather than fixed at compile time.
 *
 * The second half of the file is the transmit diagnosis: why a sendto() failed
 * (tx_fail_note()), what SHAPE a refusal storm has (enomem_note_shape()), and
 * what the radio actually emitted regardless of what sendto() accepted
 * (tx_done_cb()). servo.c renders all three onto one line every window.
 */
#include "hub.h"

#include <math.h>
#include <stdlib.h>

#include "esp_private/wifi.h"
#include "nvs_flash.h"
#include "visualiser.h"

/**
 * @brief Count satellites arriving and leaving, and keep the send list honest.
 *
 * Counting was the original reason: a satellite dropping off is invisible
 * otherwise -- the driver logs it, but nothing accumulates it, so a link that
 * flaps once an hour over an evening looks identical to one that never does.
 *
 * Dropping a departed station from the send list is the other half. The
 * counter alone left this unit unicasting audio and analysis frames at a
 * station that had gone, for a whole CLIENT_TIMEOUT_US, exhausting the WiFi
 * driver's buffer pool.
 *
 * @param arg   Unused; the event-handler signature.
 * @param base  WIFI_EVENT or IP_EVENT.
 * @param id    The event within that base.
 * @param data  The event's own payload struct.
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
         * station -- which is what lets client_joined() seed ARP. Not
         * WIFI_EVENT_AP_STACONNECTED: a station is associated before it has an
         * address, and the send list is keyed by one. */
        const ip_event_assigned_ip_to_client_t *ev = data;
        if (ev) {
            client_joined(ev->mac, &ev->ip);
        }
    }
}

#if CONFIG_DANCEFLOOR_WIFI_CHANNEL == 0
/** @brief 2.4 GHz channels sit 5 MHz apart and are 22 MHz wide, so anything
 *         within four of a candidate lands on top of it. */
#define CHANNEL_OVERLAP 4

/** @brief Where the survey lands if it cannot run at all: one of the
 *         non-overlapping three, so a fallback boot behaves like a pinned
 *         one rather than like nothing at all. */
#define CHANNEL_FALLBACK 11

/**
 * @brief Convert a linear power sum back to dBm, for printing only.
 *
 * @param mw  Summed linear power, arbitrary units.
 * @return dBm, or a floor for an untouched channel rather than the -inf
 *         log10(0) would give.
 */
static int occupancy_dbm(float mw)
{
    return mw > 0.0f ? (int)lroundf(10.0f * log10f(mw)) : -100;
}

/**
 * @brief How busy a channel actually is, which a beacon scan cannot answer.
 *
 * survey_channel()'s beacon scan ranks by summed beacon POWER, and a beacon
 * says a network is present, not that it is busy -- ten idle neighbours
 * outrank one saturated one on that metric.
 *
 * A promiscuous dwell counts EVERY frame on the air, not just beacons, and
 * rx_ctrl carries what each one cost: sig_len, and either a legacy rate or an
 * MCS. Summed over the dwell that is airtime -- occupancy, in the unit that
 * matters.
 *
 * It also subsumes the overlap problem the wide channel scan exists for: the
 * radio's receive bandwidth is ~20 MHz, so a dwell on one channel hears an
 * adjacent one's traffic natively. This measures what THIS radio receives on
 * that channel, adjacent energy included, rather than modelling it from a
 * channel-number distance.
 *
 * FCSFAIL is enabled deliberately, against IDF's "do not open it in general":
 * a frame that failed FCS still occupied the air, and collisions and marginal
 * interferers are precisely what a beacon scan cannot see. The callback is a
 * few adds, so the flood is affordable for a fraction of a second.
 */
#define OCCUPANCY_DWELL_MS 300
/** @brief Dwells per candidate, interleaved. See occupancy_survey(). */
#define OCCUPANCY_ROUNDS      3
/** @brief Preamble + IFS + ACK, charged flat per frame. */
#define OCCUPANCY_OVERHEAD_US 50

/** @brief Airtime accumulated by occupancy_cb() during the current dwell. */
static volatile uint32_t s_occ_busy_us;
/** @brief Frames occupancy_cb() saw during the current dwell. */
static volatile uint32_t s_occ_frames;

/**
 * @brief Legacy PHY rates in tenths of a Mbit/s, so the whole table is
 *        integers and the division in occupancy_cb() cannot land on a float.
 *
 * Index is the 5-bit rx_ctrl.rate for non-HT frames. The order is the enum's,
 * not ascending -- see esp_wifi_types_generic.h.
 */
static const uint16_t legacy_rate_tenths[16] = {
     10,  20,  55, 110,      /* 1, 2, 5.5, 11 long preamble          */
     10,  20,  55, 110,      /* 0x04 unused; 2, 5.5, 11 short        */
    480, 240, 120,  60,      /* 48, 24, 12, 6                        */
    540, 360, 180,  90,      /* 54, 36, 18, 9                        */
};

/** @brief MCS0..7 at 20 MHz, long GI, tenths of a Mbit/s. */
static const uint16_t ht_mcs_tenths[8] = { 65, 130, 195, 260, 390, 520, 585, 650 };

/**
 * @brief Charge one received frame's airtime to the current dwell.
 *
 * No noise floor is taken here, and it was tried: rx_ctrl.noise_floor reads
 * one constant value on every frame and every channel on this path. It is not
 * populated meaningfully, and a constant that looks like a measurement is
 * worse than no measurement.
 *
 * @param buf   The promiscuous packet, headed by its wifi_pkt_rx_ctrl_t.
 * @param type  Frame class; unused, every class counts the same here.
 */
static void occupancy_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    (void)type;
    const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
    const wifi_pkt_rx_ctrl_t *c = &p->rx_ctrl;

    uint32_t tenths;
    if (c->sig_mode == 1) {                       /* HT */
        tenths = ht_mcs_tenths[c->mcs & 7];
        if (c->cwb) {
            tenths *= 2;                          /* 40 MHz */
        }
        if (c->sgi) {
            tenths = tenths * 10 / 9;             /* short guard interval */
        }
    } else {
        tenths = legacy_rate_tenths[c->rate & 15];
    }
    if (!tenths) {
        tenths = 10;                              /* never divide by zero */
    }
    /* bits * 10 / tenths-of-Mbit = microseconds. */
    s_occ_busy_us += (uint32_t)c->sig_len * 8u * 10u / tenths + OCCUPANCY_OVERHEAD_US;
    s_occ_frames++;
}

/**
 * @brief Dwell on one channel and measure how much of it was occupied.
 *
 * @param channel        The channel to sit on.
 * @param[in,out] frames Accumulator; the dwell's frame count is added to it.
 * @return Permille of the dwell that the air was occupied.
 */
static uint32_t occupancy_dwell(int channel, uint32_t *frames)
{
    s_occ_busy_us = 0;
    s_occ_frames = 0;
    esp_wifi_set_channel((uint8_t)channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(true);
    vTaskDelay(pdMS_TO_TICKS(OCCUPANCY_DWELL_MS));
    esp_wifi_set_promiscuous(false);
    *frames += s_occ_frames;
    return s_occ_busy_us / OCCUPANCY_DWELL_MS;     /* us / ms = permille */
}

/**
 * @brief The worst each candidate gets, sampled round-robin.
 *
 * Taking the MAXIMUM, not the mean, and that is the point. A mean answers "how
 * busy is this channel typically"; what ruins audio is how bad it gets, and a
 * channel quiet most of the time and saturated for the rest is exactly the one
 * to avoid.
 *
 * Interleaved, because consecutive dwells on one channel would charge a
 * passing burst entirely to whichever channel happened to be under the
 * receiver at the time. Round-robin spreads any single burst across all three.
 *
 * More than one round, because a single dwell can catch a bursty channel in a
 * quiet gap and pick it. Where one candidate's readings swing between rounds
 * while the others hold steady, that VARIANCE is itself the finding.
 *
 * @param cand             The three candidate channels.
 * @param[out] busy_max    Worst round per candidate, permille.
 * @param[out] frames      Total frames seen per candidate, across all rounds.
 */
static void occupancy_survey(const int *cand, uint32_t *busy_max, uint32_t *frames)
{
    const wifi_promiscuous_filter_t filt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_CTRL
                     | WIFI_PROMIS_FILTER_MASK_DATA | WIFI_PROMIS_FILTER_MASK_MISC
                     | WIFI_PROMIS_FILTER_MASK_FCSFAIL,
    };
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(occupancy_cb);
    for (int k = 0; k < 3; k++) {
        busy_max[k] = 0;
        frames[k] = 0;
    }
    for (int round = 0; round < OCCUPANCY_ROUNDS; round++) {
        for (int k = 0; k < 3; k++) {
            const uint32_t b = occupancy_dwell(cand[k], &frames[k]);
            if (b > busy_max[k]) {
                busy_max[k] = b;
            }
        }
    }
    esp_wifi_set_promiscuous_rx_cb(NULL);
}

/**
 * @def OCCUPANCY_VETO_PERMILLE
 * @brief Occupancy above which a candidate loses to a quieter-measuring one,
 *        whatever the network counts say.
 *
 * Ranking is on NETWORK COUNT, with occupancy as a veto, and that ordering is
 * deliberate. Occupancy is the better question and the worse measurement: the
 * harmful traffic is intermittent, so two boots a minute apart can disagree
 * and no sample taken at boot can see traffic that starts an hour later.
 * Network count is the signal that holds still across a session.
 *
 * So count decides, and occupancy only vetoes: a candidate carrying real
 * measured traffic loses even with fewer networks, because that traffic is
 * happening now and the count is a prediction. Set well above the level the
 * quiet channels idle at, so it fires on congestion rather than on sampling
 * noise.
 */
/**
 * @brief Which channel to be an AP on, decided once, at boot.
 *
 * The channel has to be fixed for the session -- Bluetooth's adaptive
 * frequency hopping routes around a known interferer far better than a moving
 * one, and bt_bridge carries the A2DP source over the same air -- but nothing
 * in that argument says it has to be fixed at COMPILE time, and this floor
 * changes venue.
 *
 * All thirteen channels are scanned, not just the three candidates: networks
 * sitting between the candidates land on top of them (CHANNEL_OVERLAP), and a
 * scan restricted to 1/6/11 reads a channel as clear when its neighbours are
 * the largest contributor to its noise. Power is summed in LINEAR units,
 * because adding dBm figures is meaningless; occupancy_dbm() converts back
 * only to print.
 *
 * @return The channel to bring the SoftAP up on.
 */
static int survey_channel(void)
{
    static const int cand[3] = {1, 6, 11};

    /* Say so on the LED: from outside the board this is seconds of looking
     * exactly like a hang -- no AP yet, no audio, nothing on the pin. The
     * blink stopping is the ready signal. A no-op on a build without the
     * marker, and it leaves the pin dark, which is where visualiser_start()
     * puts it anyway. */
    visualiser_marker_busy(true);

    /* Scanning needs STA mode and a started radio. No STA netif is created for
     * it: a scan wants no IP stack, and the AP netif is untouched. */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* PASSIVE: this unit has no business transmitting probe requests into a
     * room it is about to be the AP of. At IDF's own default dwell
     * (WIFI_PASSIVE_SCAN_DEFAULT_TIME) each channel gets several beacon
     * intervals, which is what it takes for both the count and the power sum
     * to be sampled rather than guessed. Thirteen channels of it is a few
     * seconds, spent once at boot. */
    const wifi_scan_config_t sc = {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_PASSIVE,
        .scan_time = { .passive = WIFI_PASSIVE_SCAN_DEFAULT_TIME },
    };
    esp_err_t err = esp_wifi_scan_start(&sc, true);

    float power[3] = {0};
    int nets[3] = {0};
    int seen = 0;

    if (err == ESP_OK) {
        /* One record at a time, which frees each as it goes -- the bulk call
         * would want an array sized for a band this unit has not seen yet, and
         * internal RAM here is the scarce pool. Draining to ESP_FAIL is also
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

    /* Occupancy, per candidate, while the radio is still up. Skipped entirely
     * if the scan failed -- there is nothing to rank. */
    uint32_t busy[3] = {0}, frames[3] = {0};
    if (err == ESP_OK) {
        occupancy_survey(cand, busy, frames);
    }

    ESP_ERROR_CHECK(esp_wifi_stop());

    if (err != ESP_OK) {
        /* Loud, because the quiet version of this is a board sitting on an
         * unmeasured channel while the log implies one was chosen. */
        ESP_LOGE(TAG, "channel survey FAILED (%s) -- using ch %d unmeasured; "
                      "the band was never looked at",
                 esp_err_to_name(err), CHANNEL_FALLBACK);
        visualiser_marker_busy(false);
        return CHANNEL_FALLBACK;
    }

#define OCCUPANCY_VETO_PERMILLE 150

    int best = 0;
    for (int k = 1; k < 3; k++) {
        if (nets[k] < nets[best]) {
            best = k;
        }
    }
    if (busy[best] >= OCCUPANCY_VETO_PERMILLE) {
        for (int k = 0; k < 3; k++) {
            if (busy[k] < busy[best]) {
                ESP_LOGW(TAG, "ch%d has fewest networks (%d) but measured %"
                              PRIu32 " permille busy; taking ch%d at %" PRIu32
                              " instead",
                         cand[best], nets[best], busy[best], cand[k], busy[k]);
                best = k;
            }
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

    /* Second line rather than a longer first one: the line above is the wire
     * format the capture tooling already parses, and appending to it would put
     * the figure that DECIDES the choice after the one that no longer does. */
    ESP_LOGW(TAG, "channel occupancy: dwell %d ms x %d rounds | "
                  "ch1-busy %" PRIu32 " ch1-frames %" PRIu32 " | "
                  "ch6-busy %" PRIu32 " ch6-frames %" PRIu32 " | "
                  "ch11-busy %" PRIu32 " ch11-frames %" PRIu32
                  " -- busy is the WORST round, permille; nets chose ch%d"
                  " unless vetoed at %d",
             OCCUPANCY_DWELL_MS, OCCUPANCY_ROUNDS,
             busy[0], frames[0],
             busy[1], frames[1],
             busy[2], frames[2],
             cand[best], OCCUPANCY_VETO_PERMILLE);
    visualiser_marker_busy(false);
    return cand[best];
}
#endif /* CONFIG_DANCEFLOOR_WIFI_CHANNEL == 0 */

/* Defined beside the counters it feeds, far below; declared here because
 * wifi_start_ap() registers it. */
static void tx_done_cb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus);

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
     * The fields below are set explicitly rather than left at zero from the
     * initialiser: dtim_period has a documented range of 1-10 and zero is
     * invalid, and pmf_cfg left unset makes some clients unhappy during the
     * WPA2 handshake -- which surfaces as "incorrect password" rather than as
     * anything pointing at the real cause.
     */
    wifi_config_t wc = {0};
    strcpy((char *)wc.ap.ssid, AP_SSID);
    strcpy((char *)wc.ap.password, AP_PASS);
    wc.ap.ssid_len = strlen(AP_SSID);

    /* One of the three limits that have to agree; see MAX_CLIENTS, which is
     * the same number and carries the reasoning. The driver's own ceiling is
     * ESP_WIFI_MAX_CONN_NUM. */
    wc.ap.max_connection = MAX_CLIENTS;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wc.ap.channel = channel;
    wc.ap.dtim_period = 1;

    /*
     * 100 TU, which is IDF's floor, IDF's default, and the only value this
     * field has ever actually held on this hub.
     *
     * A SoftAP cannot send group-addressed frames whenever it likes -- stations
     * may be asleep -- so it buffers them and releases them after each DTIM
     * beacon. dtim_period is already 1, so that is every beacon, and the
     * interval is what is left: at 100 TU (1 TU = 1024 us) the hold is
     * 102.4 ms, and TX_FRAME_PACE_US in hub.h is that number.
     *
     * The field is specified in TU, "multiples of 100, range 100 ~ 60000" --
     * see wifi_ap_config_t in esp_wifi_types_generic.h. Values outside that
     * range are DISCARDED BY THE DRIVER while esp_wifi_set_config() still
     * returns ESP_OK, which says only that IDF does not validate the field on
     * the way in. The read-back below is what reports which value was kept.
     *
     * And it cannot be shortened: 100 TU is the bottom of the documented range
     * and dtim_period is already at its minimum, so the hold is a FIXED cost
     * that the group burst had to be designed against rather than tuned away.
     * Power save is not a consideration -- every satellite sets WIFI_PS_NONE
     * -- and that does not help: the AP buffers group frames for DTIM
     * regardless of whether any station is sleeping.
     */
    wc.ap.beacon_interval = 100;
    /* Matches ESP-IDF's own softAP example. Setting capable=true is a
     * deviation that some clients refuse, so leave it alone. */
    wc.ap.pmf_cfg.required = false;

#if CONFIG_DANCEFLOOR_AP_OPEN
    wc.ap.authmode = WIFI_AUTH_OPEN;
    wc.ap.password[0] = '\0';
    ESP_LOGW(TAG, "AP is OPEN (no password) -- diagnostic build");
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));

    /*
     * What the driver ACTUALLY HOLDS, read back rather than assumed. See the
     * beacon_interval note above for the failure it exists to catch: the
     * driver silently discards an out-of-range value and ESP_ERROR_CHECK
     * passes anyway.
     *
     * Kept as a regression guard on the numbers derived from that hold --
     * TX_FRAME_PACE_US in hub.h, the buffer count in sdkconfig.defaults. A
     * status field would be wrong: it cannot change while running.
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
     * Left alone, this AP negotiates HT40 and stations join at 40 MHz, which
     * puts the secondary channel far enough away that the AP occupies roughly
     * the whole 2.4 GHz band and collides with every other network in it.
     * Nothing here can use the width -- the traffic is small datagrams at a
     * fixed rate, limited by transmit opportunities rather than by bits per
     * symbol -- so it was paying the full interference cost of HT40 for none
     * of its throughput. Halving the occupied spectrum is the cheapest thing
     * available that reduces how often the channel is busy when this unit
     * wants it.
     *
     * Must follow esp_wifi_set_config(), which resets the bandwidth to the
     * default. Not asserted: it is a mitigation, not a requirement, and a
     * build that cannot set it should still stream.
     *
     * WIFI_BW20, not WIFI_BW_HT20 -- IDF 6 removed the older spelling.
     */
    const esp_err_t bw = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);
    ESP_LOGW(TAG, "AP bandwidth set to HT20: %s", esp_err_to_name(bw));

#if CONFIG_DANCEFLOOR_DISABLE_PMF
    /*
     * Turn off Protected Management Frames, because its Secure Association
     * teardown was disconnecting our own satellite: the AP starts an SA Query,
     * the satellite does not answer, and the AP disassociates it. The
     * satellite never noticed -- it counted zero disconnects while this unit
     * counted them, which is why n_sta_left exists at all.
     *
     * pmf_cfg.capable is deprecated in IDF 6 ("set to true internally"), so it
     * cannot be used to opt out. esp_wifi_disable_pmf_config() is the
     * supported way, and it must come after esp_wifi_set_config() and before
     * esp_wifi_start(). It fails on a WPA3 or WPA2/WPA3-mixed SoftAP; this one
     * is WPA2-PSK, so it applies.
     *
     * What is given up is protection of management frames -- spoofed
     * deauth/disassoc. Data stays encrypted under WPA2-PSK and the password is
     * unchanged. For a closed floor with two boards that is a poor trade
     * against losing a speaker.
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

    /* Must follow esp_wifi_start(): the driver rejects it before that. Not
     * fatal if it fails -- the hub still plays, it just loses the one stall
     * signal a cache queue cannot hide. See tx_done_cb(). */
    const esp_err_t td = esp_wifi_set_tx_done_cb(tx_done_cb);
    if (td != ESP_OK) {
        ESP_LOGW(TAG, "no tx-done callback: %s -- air-gap-max will read 0",
                 esp_err_to_name(td));
    }

    /*
     * TX power is left at the driver default. It was once capped to stop this
     * radio swamping the Bluetooth receiver when both shared one chip; the
     * two-chip split removed that need, and the cap did real harm by denying
     * rate adaptation the SNR margin it needs.
     */
#if CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS > 0
    /*
     * Pinning the PHY rate exists for a reason that no longer applies, and is
     * kept only as a diagnostic knob. Set the rate to 0 to leave it alone.
     *
     * It was mandatory while audio went out by multicast: group-addressed
     * frames fall back to the basic rate, where the packet rate this hub
     * offers physically cannot fit in the airtime available, and half the
     * audio was dropped in the queue. Unicast has no such fallback -- it uses
     * rate adaptation, which is what fixed the losses a fixed rate caused.
     *
     * Note esp_wifi_internal_set_fix_rate() applies to ALL transmission on the
     * interface, so any non-zero value here now pins unicast and disables that
     * adaptation.
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

/** @brief Distinct errno values the tally keeps side by side; the rest
 *         fall into s_tx_err_other. */
#define TX_ERR_SLOTS 4

/**
 * @brief The errno tally behind the status line's tx-fail figure.
 *
 * WHY a sendto() failed, not just how often. A non-zero tx-fail has at least
 * two completely different causes with completely different fixes, and the
 * count cannot tell them apart:
 *
 *   ENOMEM        the WiFi driver is out of TX buffers -- a load or memory
 *                 problem, since TX buffers must be internal RAM.
 *   EHOSTUNREACH  lwIP has no ARP entry and the pending-ARP queue is dropping
 *                 its overflow -- a JOIN problem, and precisely what the ARP
 *                 seeding in client_joined() exists to prevent.
 *
 * The first is fixed by taking work or memory off this board; the second by
 * looking at why a station was registered without a seeded entry.
 *
 * A tally rather than a log line per failure: these arrive in the hundreds per
 * window, and one line each would push the console out of the way of
 * everything it is meant to be read beside.
 *
 * Raced, deliberately, exactly as s_tx_fail beside it already is: three tasks
 * send and none of them take a lock. The cost of losing one is a diagnostic
 * that reads 189 instead of 190; the cost of a critical section on the send
 * path is real.
 */
static struct {
    int err;         /**< The errno this slot is counting. */
    uint32_t n;      /**< Refusals with that errno this window; 0 = free slot. */
} s_tx_err[TX_ERR_SLOTS];
/** @brief Refusals whose errno found no free slot. */
static uint32_t s_tx_err_other;

/*
 * THE SHAPE OF AN ENOMEM STORM, which is a different question from its size.
 *
 * A pool can sit exhausted because the FILL is too fast for it, or because the
 * DRAIN has stalled -- and a count of refusals looks identical either way. The
 * tell is PERIODICITY. A SoftAP releases group-addressed frames after the DTIM
 * beacon, so if the release is happening on schedule and the queue is merely
 * deeper than one release can clear, refusals arrive in clusters spaced one
 * beacon apart. If instead one burst runs continuously for hundreds of ms with
 * no spacing to speak of, the release itself has stalled.
 *
 * A stretch of refusals ENDS WHEN AN AUDIO SEND SUCCEEDS (tx_send_ok()), which
 * is the physical event of interest -- a buffer came back -- rather than a
 * timeout. A timeout cannot work here, which is worth writing down so it is
 * not tried again: audio offers packets faster than the beacon period, so
 * refusals inside a total stall are closer together than consecutive
 * beacon-released clusters are, and no threshold splits the second case
 * without also splitting the first.
 *
 * Audio is the right probe because it is the lane that is never gated --
 * fan_out() ignores the backoff by design -- so it keeps asking the pool at a
 * steady rate throughout. The frame lane would be a poor probe: it stops
 * asking the moment it is refused.
 *
 * Gauges, cleared by the window that prints them, and raced like everything
 * else on this path.
 */
static bool s_refusing;             /**< A refusal stretch is open right now. */
static int64_t s_refuse_since;      /**< When it opened. */
static int64_t s_prev_burst_at;     /**< Start of the previous stretch. */
static uint32_t n_enomem_bursts;    /**< Stretches that OPENED in this window. */
/**
 * @brief Refusals in this window, and the only thing that licenses
 *        tx_burst_summary() to print.
 *
 * Without it, a window that merely INHERITED an open stretch and then saw it
 * close with nothing refused reports a burst that did not happen -- on the
 * very window that says the hub recovered.
 */
static uint32_t n_enomem_refusals;
/** @brief A stretch was already open when this window started. */
static bool s_carried_open;
/** @brief Longest single stretch this window, us. */
static int32_t n_refuse_max_us;

/**
 * @brief Where the stretches fall relative to the beacon, as a histogram
 *        rather than a single number.
 *
 * A minimum is the wrong statistic here: it is set by the single tightest pair
 * in the window, so it reads low through any transient while the stretches are
 * in fact arriving at the beacon rate. Four fixed buckets around the 102.4 ms
 * DTIM hold make the shape legible at a glance without carrying an array of
 * samples:
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
/** @brief Stretch-to-stretch gaps this window, bucketed as above. */
static uint32_t n_burst_gap[BURST_GAP_BUCKETS];

/**
 * @brief Fold one ENOMEM refusal into the burst gauges.
 *
 * @param now  When the refusal happened, us.
 */
static void enomem_note_shape(int64_t now)
{
    n_enomem_refusals++;
    if (!s_refusing) {
        s_refusing = true;
        s_refuse_since = now;
        n_enomem_bursts++;
        if (s_prev_burst_at) {
            /* Start-to-start, not end-to-start: a beacon releases on a fixed
             * period, so it is the period between stretches that should land
             * on the beacon interval. An end-to-start gap would be that period
             * minus however long the stretch ran. */
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
 * WHAT THE RADIO ACTUALLY DID, which is a different question from what
 * sendto() accepted and the only one a cache queue cannot answer for.
 *
 * Every other hub-side stall signal is stamped at sendto() RETURN: tx-fail
 * counts a refusal, fanout-gap-max measures the gap between accepted sends,
 * lead-min is computed as the packet is built. Turn
 * ESP_WIFI_CACHE_TX_BUFFER_NUM on and the driver stops refusing -- it queues
 * in PSRAM instead -- so all three go quiet whether or not the air improved.
 *
 * esp_wifi_set_tx_done_cb() fires when a frame leaves the radio, so the gap
 * between callbacks is the stall itself, measured past every queue in front of
 * it. That is the point: it stays honest with the cache queue on.
 *
 * It also separates two causes the refusals could not. txStatus is false when
 * the frame was transmitted and never acknowledged -- retries exhausted, i.e.
 * the air. So:
 *
 *   air-gap large, txdone-fail rising ... the medium. Frames went and died.
 *   air-gap large, txdone-fail flat .... frames are not being LOST, which
 *                                        rules out retry exhaustion.
 *
 * It does not finish the job: a flat txdone-fail is equally consistent with
 * the driver never dequeuing and with a busy medium deferring us in CCA, since
 * a frame that waits for the air and then succeeds is not an ack failure.
 * Separating those needs something this callback cannot see. What it does
 * establish is that the frames which go are getting through.
 *
 * Kept to three counters and no allocation: this runs in the WiFi task's
 * context on every transmitted frame. Deliberately no IRAM_ATTR -- internal
 * DRAM is the scarce pool here and IRAM is the wrong one to spend on a counter
 * bump.
 */
static volatile uint32_t n_txdone;            /**< Frames the radio reported done. */
static volatile uint32_t n_txdone_fail;       /**< ...of which the air never acked. */
static volatile int32_t  n_air_gap_max_us;    /**< Widest silence between two of them. */
static int64_t s_txdone_prev_at;              /**< When the last one completed. */

/**
 * @brief Stamp a completed transmission; the driver's tx-done callback.
 *
 * @param ifidx     Interface the frame left on; unused, this AP has one.
 * @param data      The frame; unused, only its completion is measured.
 * @param data_len  Its length; unused, for the same reason.
 * @param txStatus  true if the air acknowledged it.
 */
static void tx_done_cb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus)
{
    (void)ifidx; (void)data; (void)data_len;
    const int64_t now = esp_timer_get_time();
    n_txdone++;
    if (!txStatus) {
        n_txdone_fail++;
    }
    if (s_txdone_prev_at) {
        const int64_t gap = now - s_txdone_prev_at;
        if (gap > n_air_gap_max_us) {
            n_air_gap_max_us = (int32_t)(gap > INT32_MAX ? INT32_MAX : gap);
        }
    }
    s_txdone_prev_at = now;
}

/* Render the air's own three numbers and clear them. ALWAYS PRINTS, unlike
 * the refusal instruments beside it: a clean window here is a positive
 * measurement -- the radio kept emitting -- and that is exactly the reading
 * the cache-queue experiment needs to be able to trust. An empty string would
 * be silence about silence. Declared in hub.h. */
void tx_air_summary(char *buf, size_t len)
{
    snprintf(buf, len, " | air-gap-max %ld ms | txdone %" PRIu32
                       " | txdone-fail %" PRIu32,
             (long)(n_air_gap_max_us / 1000), n_txdone, n_txdone_fail);
    n_air_gap_max_us = 0;
    n_txdone = 0;
    n_txdone_fail = 0;
}

/* An audio packet reached the transmit path, so the pool has a buffer again.
 * Called from fan_out() on FANOUT_SENT -- see enomem_note_shape() for why
 * audio and not any other lane is what closes a stretch. Declared in hub.h. */
void tx_send_ok(void)
{
    s_refusing = false;
}

/* Render the burst shape and clear it. Empty while nothing has been refused,
 * so a clean window's line stays byte-for-byte what it was before this
 * instrument existed -- the same rule tx_fail_summary() follows. Declared in
 * hub.h. */
void tx_burst_summary(char *buf, size_t len)
{
    if (!n_enomem_refusals) {
        buf[0] = '\0';
        /*
         * The counters still have to be cleared on this path. A retry that
         * SUCCEEDS never reaches tx_fail_note(), so it bumps n_audio_retry
         * without bumping n_enomem_refusals: a window where every ENOMEM was
         * rescued takes this early return, prints nothing, and would otherwise
         * carry those retries forward to be printed beside a LATER window's
         * refusals.
         *
         * It matters more from here on, not less: with
         * ESP_WIFI_CACHE_TX_BUFFER_NUM on, refusals are exactly what stops
         * happening, so this is the path most windows take.
         */
        n_refuse_near_frame = 0;
        n_audio_retry = 0;
        n_audio_retry_ok = 0;
        /* Nothing was refused, so nothing is claimed -- but whether a stretch
         * is open still has to reach the next window, or a stall that goes
         * quiet for one window and resumes would read as two. */
        s_carried_open = s_refusing;
        return;
    }
    /* A stretch that OPENED before this window still ran through it, and is
     * one of this window's bursts even though nothing here opened it. */
    const uint32_t bursts = n_enomem_bursts + (s_carried_open ? 1u : 0u);

    /* The buckets always print, even all-zero, and the labels are in the name:
     * `gaps <25/25-75/75-150/>150`. A single stretch produces no gap at all --
     * a gap needs two -- and four zeros say that plainly, where a single
     * figure would have to print the word "none" to avoid being read as "no
     * time between clusters", which is the opposite of what it means. */
    snprintf(buf, len, " | enomem-bursts %" PRIu32 " | refuse-max %ld ms"
                       " | gaps %" PRIu32 "/%" PRIu32 "/%" PRIu32 "/%" PRIu32
                       " | refuse-near-frame %" PRIu32
                       " | audio-retry %" PRIu32 " | audio-retry-ok %" PRIu32,
             bursts, (long)(n_refuse_max_us / 1000),
             n_burst_gap[0], n_burst_gap[1], n_burst_gap[2], n_burst_gap[3],
             n_refuse_near_frame, n_audio_retry, n_audio_retry_ok);
    /* Cleared here, with the burst shape they are read against: all of it
     * describes one window, and a counter that outlived its window would be
     * attributed to the next one's refusals. */
    n_refuse_near_frame = 0;
    n_audio_retry = 0;
    n_audio_retry_ok = 0;

    /*
     * A stretch still open at the boundary is CARRIED, not dropped. Dropping
     * it makes a multi-window stall -- the single worst thing this instrument
     * exists to catch -- print once and then report nothing at all, because no
     * NEW stretch ever begins. It would read as a hub that recovered.
     *
     * Carried as a FLAG rather than as a count: a count survives into a window
     * that refuses nothing and prints a burst that did not happen. The flag is
     * only ever honoured beside a real refusal, above.
     *
     * s_refusing and s_refuse_since survive, so refuse-max keeps growing from
     * the true start and the next window reports the stall's full length so
     * far rather than restarting the clock. s_prev_burst_at survives too: a
     * gap is a real interval whether or not a window boundary fell inside it.
     */
    s_carried_open = s_refusing;
    n_enomem_bursts = 0;
    n_enomem_refusals = 0;
    n_refuse_max_us = 0;
    memset(n_burst_gap, 0, sizeof(n_burst_gap));
}

/* Render the lane breakdown and clear it. Always non-empty -- it is printed
 * inside the existing `(N audio)` parentheses, which have never been optional.
 * Declared in hub.h. */
void tx_fail_lanes(char *buf, size_t len)
{
    static const char *const name[TX_LANE_N] = {
        "audio", "frame", "vol", "meta", "probe", "fec",
    };
    size_t off = 0;
    buf[0] = '\0';
    for (int i = 0; i < TX_LANE_N; i++) {
        const uint32_t n = s_tx_lane_fail[i];
        s_tx_lane_fail[i] = 0;
        /* AUDIO always prints, even at zero: it is the number the room cares
         * about and its absence must not read as "not measured". The rest
         * print only when they have something to say, so a clean line stays
         * short. */
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
 * tx-fail was one number shared by audio, analysis frames, ML results,
 * metadata and the log shipper, so it could not say how many satellite gaps
 * this hub had caused itself -- and that is the only part of it that is
 * audible. A refused frame costs one repaint; a refused audio packet is a hole
 * in the sound on every satellite at once, and under FEC it is recoverable
 * only if the NEXT packet gets through, which under a burst of ENOMEM is
 * exactly what does not happen.
 *
 * Kept as a named wrapper now that tx_fail_note() takes a lane: the two audio
 * send paths in timeline.c are its only callers. Declared in hub.h.
 */
void tx_fail_note_audio(int err)
{
    tx_fail_note(TX_LANE_AUDIO, err);
}

/* Count one refused send and react to it. Declared in hub.h. */
void tx_fail_note(tx_lane_t lane, int err)
{
    s_tx_fail++;
    if ((unsigned)lane < TX_LANE_N) {
        s_tx_lane_fail[lane]++;
    }

    /*
     * ENOMEM is the WiFi pool out of TX buffers, and it is the one failure a
     * non-audio lane can do something about: back off for TX_BACKOFF_US so
     * publish_frame() yields and the buffers audio is being refused are left
     * for audio. fan_out() does not check this, so audio keeps sending and, if
     * it still hits ENOMEM, re-arms the deadline from here. The errno tally
     * below stays the whole of the diagnosis; this is the reaction to it.
     */
    if (err == ENOMEM) {
        const int64_t now = esp_timer_get_time();
        enomem_note_shape(now);
        s_tx_congested_until = now + TX_BACKOFF_US;
        /* Only AUDIO asks the question, because only audio is the victim: a
         * frame refused while another frame is in flight is the lane queueing
         * behind itself, which says nothing about who starved the room. See
         * n_refuse_near_frame in hub.h. */
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

/* Render the errno tally and clear it, for the status line in servo.c. Writes
 * an EMPTY string when nothing failed, so a clean run's line is byte-for-byte
 * what it has always been and stays comparable with every log captured before
 * this instrument existed. Cleared here because the caller zeroes s_tx_fail on
 * the same line; the two must not drift apart. Declared in hub.h. */
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

/* Bring up the one UDP socket every non-audio lane and the probe server share.
 * Declared in hub.h. */
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
