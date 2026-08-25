
#include "hub.h"

#include "lwip/etharp.h"

void clients_snapshot(client_t *dst)
{
    portENTER_CRITICAL(&s_clients_lock);
    memcpy(dst, s_clients, sizeof(client_t) * MAX_CLIENTS);
    portEXIT_CRITICAL(&s_clients_lock);
}

void clients_age(int64_t now)
{
    portENTER_CRITICAL(&s_clients_lock);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (s_clients[i].last_seen &&
            now - s_clients[i].last_seen > CLIENT_TIMEOUT_US) {
            s_clients[i].last_seen = 0;
            n_sta_timeout++;
        }
    }
    portEXIT_CRITICAL(&s_clients_lock);
}

void client_seen(const struct sockaddr_in *from)
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

void client_joined(const uint8_t mac[6], const esp_ip4_addr_t *ip)
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

    streamer_send_vol(audio_volume);
    ESP_LOGI(TAG, "satellite " IPSTR " has an address -- on the send list, ARP seeded",
             IP2STR(ip));
}

void client_gone(const uint8_t mac[6])
{
    if (!s_ap_netif) {
        return;
    }

    esp_netif_pair_mac_ip_t pair;
    memcpy(pair.mac, mac, sizeof(pair.mac));
    pair.ip.addr = 0;
    if (esp_netif_dhcps_get_clients_by_mac(s_ap_netif, 1, &pair) != ESP_OK || !pair.ip.addr) {

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

    ip4_addr_t gone = { .addr = pair.ip.addr };
    (void)esp_netif_tcpip_exec(arp_drop, &gone);

    if (found) {
        n_sta_dropped++;
        ESP_LOGW(TAG, "satellite " IPSTR " disassociated -- dropped from the send list",
                 IP2STR(&pair.ip));
    }
}

void streamer_send_meta(const uint8_t *meta, uint16_t len)
{
    if (sock < 0 || len > sizeof(((meta_msg_t *)0)->payload)) {
        return;
    }
    meta_msg_t msg = { .type = MSG_META };
    memcpy(msg.payload, meta, len);

    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (snapshot[i].last_seen) {
            if (sendto(sock, &msg, sizeof(msg), 0,
                       (struct sockaddr *)&snapshot[i].addr,
                       sizeof(snapshot[i].addr)) < 0) {
                tx_fail_note(TX_LANE_META, errno);
            }
        }
    }
}

void streamer_send_vol(uint8_t volume)
{

    if (sock < 0 || !audio_vol_known) {
        return;
    }
    vol_msg_t msg = { .type = MSG_VOL, .volume = volume };

    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (snapshot[i].last_seen) {
            if (sendto(sock, &msg, sizeof(msg), 0,
                       (struct sockaddr *)&snapshot[i].addr,
                       sizeof(snapshot[i].addr)) < 0) {
                tx_fail_note(TX_LANE_VOL, errno);
            } else {
                n_vol_tx++;
            }
        }
    }
}

void streamer_set_volume(uint8_t volume)
{
    const uint8_t v = volume > AUDIO_VOL_MAX ? AUDIO_VOL_MAX : volume;
    const bool changed = (v != audio_volume) || !audio_vol_known;
    if (changed) {
        ESP_LOGW(TAG, "VOLUME %u/%d", v, AUDIO_VOL_MAX);
    }

    audio_volume = v;
    audio_vol_known = true;
    if (!changed) {
        return;
    }
    for (int i = 0; i < VOL_CHANGE_REPEATS; i++) {
        streamer_send_vol(v);
    }
}

static void vol_repeat_cb(void *arg)
{
    (void)arg;
    streamer_send_vol(audio_volume);
}

void vol_repeat_start(void)
{
    const esp_timer_create_args_t args = {
        .callback = vol_repeat_cb,
        .name = "volrpt",
    };
    esp_timer_handle_t h;
    ESP_ERROR_CHECK(esp_timer_create(&args, &h));
    ESP_ERROR_CHECK(esp_timer_start_periodic(h, VOL_REPEAT_US));
}

#if CONFIG_DANCEFLOOR_ENABLE_VISUALISER

_Static_assert(TX_FRAME_BATCH * sizeof(vis_frame_t) <= FRAME_PAYLOAD_MAX,
               "TX_FRAME_BATCH frames do not fit in frame_msg_t.payload");

void publish_frame(const vis_frame_t *f)
{

    static frame_msg_t msg = { .type = MSG_FRAME, .len = (uint8_t)sizeof(vis_frame_t) };
    static uint8_t     pending;
    static int64_t     pending_due;
    static int64_t     s_pace_at;

    const int64_t now = esp_timer_get_time();

    if (pending && pending_due > 0 && now > pending_due) {
        n_tx_pace_skip += pending;
        pending = 0;
    }

    memcpy(msg.payload + (size_t)pending * sizeof(*f), f, sizeof(*f));
    pending++;
    pending_due = f->due_us;

    if (pending < TX_FRAME_BATCH && now < s_pace_at) {
        return;
    }
    s_pace_at = now + TX_FRAME_PACE_US;

    const uint8_t count = pending;
    pending = 0;

    if (now < s_tx_congested_until) {
        n_tx_cong_skip += count;
        return;
    }
    msg.count = count;
    const size_t bytes = FRAME_MSG_BYTES((size_t)count * sizeof(*f));

    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    s_tx_frame_sent_us = now;

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!snapshot[i].last_seen) {
            continue;
        }
        if (sendto(sock, &msg, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {
            tx_fail_note(TX_LANE_FRAME, errno);
        }
    }
}
#endif
