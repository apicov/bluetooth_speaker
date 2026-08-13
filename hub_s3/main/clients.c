/*
 * The send list: who is listening, and every fan-out over them.
 *
 * Registration is implicit -- a satellite that has probed recently is listening
 * -- with two events sharpening the edges: client_joined() puts a unit on the
 * list as soon as DHCP gives it an address (and seeds the ARP entry that makes
 * the first unicast land), and client_gone() takes it off the instant it
 * disassociates rather than waiting out CLIENT_TIMEOUT_US.
 *
 * Everything that sends to satellites lives here for one reason: they all need
 * the same snapshot under the same spinlock, and that copy was written out four
 * times in streamer.c. The messages they build stay separate on purpose -- see
 * clients_snapshot() in hub.h.
 */
#include "hub.h"

/* Reached under esp_netif_tcpip_exec() only -- see the ARP note below. */
#include "lwip/etharp.h"

void clients_snapshot(client_t *dst)
{
    portENTER_CRITICAL(&s_clients_lock);
    memcpy(dst, s_clients, sizeof(client_t) * MAX_CLIENTS);
    portEXIT_CRITICAL(&s_clients_lock);
}

/*
 * Drop satellites that have stopped probing.
 *
 * This used to live inside the audio send loop and NOWHERE ELSE, which meant
 * aging only happened while audio was flowing. A satellite that walked out of
 * range during a gap between tracks -- or while the phone was paused, or before
 * the first packet of a session -- stayed on the send list indefinitely, because
 * the only code that could remove it ran only when there was something to send
 * it. The event handler covers a clean disassociation; this covers the unit that
 * vanishes without saying so, and that was the case left uncovered.
 *
 * Now called from both: before each send, so the fast path is unchanged, and
 * from the 5 s tick, so a stopped stream still ages the list. Both call it with
 * their own `now`, and it is idempotent -- a cleared slot has last_seen 0 and is
 * skipped.
 *
 * n_sta_timeout still counts once per departure, because the slot is cleared
 * here and a cleared slot never re-enters the branch.
 */
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
void client_gone(const uint8_t mac[6])
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
void publish_frame(const vis_frame_t *f)
{
    /* Yield the instant the TX pool is exhausted: see TX_BACKOFF_US. fan_out() --
     * the audio path -- is never gated, so this is what keeps a frame burst off the
     * buffers audio is being refused. */
    if (esp_timer_get_time() < s_tx_congested_until) {
        n_tx_cong_skip++;
        return;
    }
    if (sizeof(*f) > FRAME_PAYLOAD_MAX) {
        return;                              /* refuse rather than truncate */
    }
    frame_msg_t msg = { .type = MSG_FRAME, .len = (uint8_t)sizeof(*f) };
    memcpy(msg.payload, f, sizeof(*f));
    const size_t bytes = FRAME_MSG_BYTES(sizeof(*f));

    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!snapshot[i].last_seen) {
            continue;
        }
        if (sendto(sock, &msg, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {
            tx_fail_note(errno);
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
void publish_ml(const ml_result_t *r)
{
    const int64_t now = esp_timer_get_time();
    /* Yield the instant the TX pool is exhausted: see TX_BACKOFF_US. */
    if (now < s_tx_congested_until) {
        n_tx_cong_skip++;
        return;
    }
    /* Capped well under the analysis rate -- see ML_PUBLISH_PERIOD_US. The comment
     * above this function named this rate limit as the first thing the lane would
     * grow; this is it. A stale result costs a satellite one old reading, not a gap. */
    static int64_t last_us = 0;
    if (now - last_us < ML_PUBLISH_PERIOD_US) {
        n_ml_throttled++;
        return;
    }
    last_us = now;
    if (sizeof(*r) > ML_PAYLOAD_MAX) {
        return;                              /* refuse rather than truncate */
    }
    ml_msg_t msg = { .type = MSG_ML, .len = (uint8_t)sizeof(*r) };
    memcpy(msg.payload, r, sizeof(*r));
    const size_t bytes = ML_MSG_BYTES(sizeof(*r));

    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!snapshot[i].last_seen) {
            continue;
        }
        if (sendto(sock, &msg, bytes, 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr)) < 0) {
            tx_fail_note(errno);
        }
    }
}
#endif
