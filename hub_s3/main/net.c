
#include "hub.h"

#include <math.h>
#include <stdlib.h>

#include "esp_private/wifi.h"
#include "nvs_flash.h"
#include "visualiser.h"

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

        const ip_event_assigned_ip_to_client_t *ev = data;
        if (ev) {
            client_joined(ev->mac, &ev->ip);
        }
    }
}

#if CONFIG_DANCEFLOOR_WIFI_CHANNEL == 0

#define CHANNEL_OVERLAP 4

#define CHANNEL_FALLBACK 11

static int occupancy_dbm(float mw)
{
    return mw > 0.0f ? (int)lroundf(10.0f * log10f(mw)) : -100;
}

#define OCCUPANCY_DWELL_MS 300
#define OCCUPANCY_ROUNDS      3
#define OCCUPANCY_OVERHEAD_US 50

static volatile uint32_t s_occ_busy_us;
static volatile uint32_t s_occ_frames;

static const uint16_t legacy_rate_tenths[16] = {
     10,  20,  55, 110,
     10,  20,  55, 110,
    480, 240, 120,  60,
    540, 360, 180,  90,
};

static const uint16_t ht_mcs_tenths[8] = { 65, 130, 195, 260, 390, 520, 585, 650 };

static void occupancy_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    (void)type;
    const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
    const wifi_pkt_rx_ctrl_t *c = &p->rx_ctrl;

    uint32_t tenths;
    if (c->sig_mode == 1) {
        tenths = ht_mcs_tenths[c->mcs & 7];
        if (c->cwb) {
            tenths *= 2;
        }
        if (c->sgi) {
            tenths = tenths * 10 / 9;
        }
    } else {
        tenths = legacy_rate_tenths[c->rate & 15];
    }
    if (!tenths) {
        tenths = 10;
    }

    s_occ_busy_us += (uint32_t)c->sig_len * 8u * 10u / tenths + OCCUPANCY_OVERHEAD_US;
    s_occ_frames++;
}

static uint32_t occupancy_dwell(int channel, uint32_t *frames)
{
    s_occ_busy_us = 0;
    s_occ_frames = 0;
    esp_wifi_set_channel((uint8_t)channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(true);
    vTaskDelay(pdMS_TO_TICKS(OCCUPANCY_DWELL_MS));
    esp_wifi_set_promiscuous(false);
    *frames += s_occ_frames;
    return s_occ_busy_us / OCCUPANCY_DWELL_MS;
}

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

static int survey_channel(void)
{
    static const int cand[3] = {1, 6, 11};

    visualiser_marker_busy(true);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

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

    uint32_t busy[3] = {0}, frames[3] = {0};
    if (err == ESP_OK) {
        occupancy_survey(cand, busy, frames);
    }

    ESP_ERROR_CHECK(esp_wifi_stop());

    if (err != ESP_OK) {

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

    ESP_LOGW(TAG, "channel survey: nets-seen %d | ch1-dbm %d ch1-nets %d | "
                  "ch6-dbm %d ch6-nets %d | ch11-dbm %d ch11-nets %d | chose %d",
             seen,
             occupancy_dbm(power[0]), nets[0],
             occupancy_dbm(power[1]), nets[1],
             occupancy_dbm(power[2]), nets[2],
             cand[best]);

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
#endif

static void tx_done_cb(uint8_t ifidx, uint8_t *data, uint16_t *data_len, bool txStatus);

void wifi_start_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_ap_netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                       WIFI_EVENT_AP_STADISCONNECTED,
                                                       wifi_event, NULL, NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                       IP_EVENT_ASSIGNED_IP_TO_CLIENT,
                                                       wifi_event, NULL, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

#if CONFIG_DANCEFLOOR_WIFI_CHANNEL == 0
    const int channel = survey_channel();
#else
    const int channel = CONFIG_DANCEFLOOR_WIFI_CHANNEL;
    ESP_LOGW(TAG, "channel pinned to %d, no survey -- runs that must compare "
                  "with each other want this", channel);
#endif

    wifi_config_t wc = {0};
    strcpy((char *)wc.ap.ssid, AP_SSID);
    strcpy((char *)wc.ap.password, AP_PASS);
    wc.ap.ssid_len = strlen(AP_SSID);

    wc.ap.max_connection = MAX_CLIENTS;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wc.ap.channel = channel;
    wc.ap.dtim_period = 1;

    wc.ap.beacon_interval = 100;

    wc.ap.pmf_cfg.required = false;

#if CONFIG_DANCEFLOOR_AP_OPEN
    wc.ap.authmode = WIFI_AUTH_OPEN;
    wc.ap.password[0] = '\0';
    ESP_LOGW(TAG, "AP is OPEN (no password) -- diagnostic build");
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));

    wifi_config_t back = {0};
    if (esp_wifi_get_config(WIFI_IF_AP, &back) == ESP_OK) {
        ESP_LOGW(TAG, "AP beacon_interval %u TU (asked %u), dtim_period %u "
                      "(asked %u) -- any group frame is held for beacon x dtim",
                 (unsigned)back.ap.beacon_interval, (unsigned)wc.ap.beacon_interval,
                 (unsigned)back.ap.dtim_period, (unsigned)wc.ap.dtim_period);
    } else {
        ESP_LOGW(TAG, "AP config read-back failed -- beacon/DTIM hold unknown");
    }

    const esp_err_t bw = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW20);
    ESP_LOGW(TAG, "AP bandwidth set to HT20: %s", esp_err_to_name(bw));

#if CONFIG_DANCEFLOOR_DISABLE_PMF

    const esp_err_t pmf = esp_wifi_disable_pmf_config(WIFI_IF_AP);
    ESP_LOGW(TAG, "PMF disabled on the AP: %s", esp_err_to_name(pmf));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    const esp_err_t td = esp_wifi_set_tx_done_cb(tx_done_cb);
    if (td != ESP_OK) {
        ESP_LOGW(TAG, "no tx-done callback: %s -- air-gap-max will read 0",
                 esp_err_to_name(td));
    }

#if CONFIG_DANCEFLOOR_WIFI_PHY_RATE_MBPS > 0

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

    ESP_LOGW(TAG, "PHY rate adaptation is ON (rate not pinned)");
#endif
    ESP_LOGI(TAG, "SoftAP \"%s\" pass \"%s\" ch %d, radio at defaults",
             AP_SSID, AP_PASS, channel);
}

#define TX_ERR_SLOTS 4
static struct {
    int err;
    uint32_t n;
} s_tx_err[TX_ERR_SLOTS];
static uint32_t s_tx_err_other;

static bool s_refusing;
static int64_t s_refuse_since;
static int64_t s_prev_burst_at;
static uint32_t n_enomem_bursts;

static uint32_t n_enomem_refusals;
static bool s_carried_open;
static int32_t n_refuse_max_us;

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

static volatile uint32_t n_txdone;
static volatile uint32_t n_txdone_fail;
static volatile int32_t  n_air_gap_max_us;
static int64_t s_txdone_prev_at;

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

void tx_air_summary(char *buf, size_t len)
{
    snprintf(buf, len, " | air-gap-max %ld ms | txdone %" PRIu32
                       " | txdone-fail %" PRIu32,
             (long)(n_air_gap_max_us / 1000), n_txdone, n_txdone_fail);
    n_air_gap_max_us = 0;
    n_txdone = 0;
    n_txdone_fail = 0;
}

void tx_send_ok(void)
{
    s_refusing = false;
}

void tx_burst_summary(char *buf, size_t len)
{
    if (!n_enomem_refusals) {
        buf[0] = '\0';

        n_refuse_near_frame = 0;
        n_audio_retry = 0;
        n_audio_retry_ok = 0;

        s_carried_open = s_refusing;
        return;
    }

    const uint32_t bursts = n_enomem_bursts + (s_carried_open ? 1u : 0u);

    snprintf(buf, len, " | enomem-bursts %" PRIu32 " | refuse-max %ld ms"
                       " | gaps %" PRIu32 "/%" PRIu32 "/%" PRIu32 "/%" PRIu32
                       " | refuse-near-frame %" PRIu32
                       " | audio-retry %" PRIu32 " | audio-retry-ok %" PRIu32,
             bursts, (long)(n_refuse_max_us / 1000),
             n_burst_gap[0], n_burst_gap[1], n_burst_gap[2], n_burst_gap[3],
             n_refuse_near_frame, n_audio_retry, n_audio_retry_ok);

    n_refuse_near_frame = 0;
    n_audio_retry = 0;
    n_audio_retry_ok = 0;

    s_carried_open = s_refusing;
    n_enomem_bursts = 0;
    n_enomem_refusals = 0;
    n_refuse_max_us = 0;
    memset(n_burst_gap, 0, sizeof(n_burst_gap));
}

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

        if (!n && i != TX_LANE_AUDIO) {
            continue;
        }
        off += snprintf(buf + off, off < len ? len - off : 0, "%s%" PRIu32 " %s",
                        off ? ", " : "", n, name[i]);
    }
}

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

    if (err == ENOMEM) {
        const int64_t now = esp_timer_get_time();
        enomem_note_shape(now);
        s_tx_congested_until = now + TX_BACKOFF_US;

        if (lane == TX_LANE_AUDIO
            && now - s_tx_frame_sent_us <= TX_NEAR_US) {
            n_refuse_near_frame++;
        }
    }
    for (int i = 0; i < TX_ERR_SLOTS; i++) {
        if (s_tx_err[i].n == 0) {
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
