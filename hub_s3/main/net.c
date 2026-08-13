/*
 * Bringing up the radio and the socket everything else talks over.
 *
 * The SoftAP configuration is mostly a list of things that were measured going
 * wrong: HT20 because the driver negotiates HT40 and nothing here can use it,
 * PMF off because its teardown was disassociating our own satellite, power save
 * off because it parks the radio between beacons.
 */
#include "hub.h"

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

#if CONFIG_DANCEFLOOR_AUDIO_MCAST
    /*
     * Audio goes out by multicast -- see fan_out() in timeline.c. The one thing
     * that made that unworkable before was the group frame rate: group-addressed
     * frames transmit at the lowest basic rate, which is 1 Mbps while 11b is in
     * the basic set, and at 1 Mbps a ~1 kB SBC packet costs 8.3 ms -- the stream
     * needs more airtime than a second has, so half of it was dropped in the
     * queue (net.c history, ~20% loss measured at every rate tried).
     *
     * Dropping 11b from the AP protocol leaves 11g/11n, whose basic rates start
     * at 6 Mbps. That is an OFDM rate: the same ~1 kB packet costs ~1.4 ms, the
     * stream fits with room to spare, and unicast keeps its g/n rate adaptation
     * (this is not esp_wifi_internal_set_fix_rate(), which pins ALL TX and was
     * removed for the 23% loss disabling adaptation caused). Must precede start.
     *
     * esp_wifi_config_80211_tx() (esp_wifi.h, IDF 6) is NOT a fallback for this.
     * Its wifi_tx_rate_config_t has no multicast/broadcast field -- it pins the
     * rate for the whole AP interface, unicast included, which disables rate
     * adaptation. That is the 23% loss esp_wifi_internal_set_fix_rate() caused
     * before it was removed, applied here all over again. Dropping 11b via
     * set_protocol() is the fix that moves the group rate without touching
     * unicast adaptation; there is no second lever, so the group rate must be
     * confirmed at 6 Mbps another way -- by the loss floor, not the air rate.
     */
    const esp_err_t proto = esp_wifi_set_protocol(
        WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    ESP_LOGW(TAG, "multicast audio: AP protocol 11G|11N (11b dropped, group rate "
                  "->6 Mbps): %s", esp_err_to_name(proto));
#endif

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

void tx_fail_note(int err)
{
    s_tx_fail++;
    /*
     * ENOMEM is the WiFi pool out of TX buffers, and it is the one failure a
     * non-audio lane can do something about: back off for TX_BACKOFF_US so
     * publish_frame/publish_ml yield and the buffers audio is being refused are
     * left for audio. fan_out() does not check this, so audio keeps sending and,
     * if it still hits ENOMEM, re-arms the deadline from here. The errno tally
     * above stays the whole of the diagnosis; this is the reaction to it.
     */
    if (err == ENOMEM) {
        s_tx_congested_until = esp_timer_get_time() + TX_BACKOFF_US;
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
