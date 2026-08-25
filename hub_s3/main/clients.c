/**
 * @file clients.c
 * @brief The send list -- who is listening -- and every fan-out over them.
 *
 * Registration is implicit: a satellite that has probed recently is
 * listening. Two events sharpen the edges: client_joined() puts a unit on
 * the list the moment DHCP gives it an address (and seeds the ARP entry
 * that makes its first unicast land), and client_gone() takes it off the
 * instant it disassociates rather than waiting out CLIENT_TIMEOUT_US.
 *
 * Everything that sends to satellites lives here because all of it needs
 * the same snapshot under the same spinlock. The messages they build stay
 * separate on purpose -- see clients_snapshot() in hub.h.
 */
#include "hub.h"

/* For etharp_add/remove_static_entry() -- reached via esp_netif_tcpip_exec()
 * only; see the ARP note at client_joined(). */
#include "lwip/etharp.h"

void clients_snapshot(client_t *dst)
{
    portENTER_CRITICAL(&s_clients_lock);
    memcpy(dst, s_clients, sizeof(client_t) * MAX_CLIENTS);
    portEXIT_CRITICAL(&s_clients_lock);
}

/*
 * Called from two places: before each audio send, and from the 5 s tick.
 * Both are needed -- aging only on the send path would mean a satellite
 * that vanished while nothing was playing (between tracks, before the first
 * packet, while the phone is paused) stayed on the list indefinitely. The
 * WiFi event handler covers a clean disassociation; this covers the unit
 * that goes silent without saying so.
 *
 * Idempotent: a cleared slot has last_seen 0 and is skipped, so n_sta_timeout
 * counts each departure exactly once.
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
 * The ARP entries that ride alongside the send list.
 *
 * They must be seeded, because the DHCP server removes its own static entry
 * for the address BEFORE raising the event that reports the assignment --
 * the callback fires at the one instant the entry is guaranteed absent, and
 * nothing repopulates it until the station itself transmits (its ARP
 * request, immediately before its first probe). Registering a satellite on
 * the event without seeding her ARP entry is worse than not registering
 * her: every unicast to her piles into lwIP's pending-ARP queue, which
 * drops its overflow. Seeding from the event's own MAC needs no lease
 * lookup and cannot misidentify the station.
 *
 * Via esp_netif_tcpip_exec(): etharp_add_static_entry() asserts the lwIP
 * core is locked, and this build does not run TCPIP core locking, so a
 * direct call from the event task is wrong however well it appears to work.
 *
 * Static entries never age out, and the disassociation path below removes
 * one only when it can resolve the lease. That leak is bounded by the DHCP
 * pool -- re-seeding an address overwrites its own slot, so at most
 * MAX_CLIENTS entries against ARP_TABLE_SIZE -- and harmless: the send list
 * gates sending, not the ARP table.
 */
typedef struct {
    ip4_addr_t ip;
    struct eth_addr mac;
} arp_seed_t;

/**
 * @brief esp_netif_tcpip_exec() thunk: seed one static ARP entry.
 * @param ctx  The arp_seed_t to install.
 * @return ESP_OK if lwIP took the entry.
 */
static esp_err_t arp_add(void *ctx)
{
    arp_seed_t *s = ctx;
    return etharp_add_static_entry(&s->ip, &s->mac) == ERR_OK ? ESP_OK : ESP_FAIL;
}

/**
 * @brief esp_netif_tcpip_exec() thunk: drop one static ARP entry.
 * @param ctx  The ip4_addr_t to remove.
 * @return ESP_OK if lwIP dropped the entry.
 */
static esp_err_t arp_drop(void *ctx)
{
    return etharp_remove_static_entry((const ip4_addr_t *)ctx) == ERR_OK ? ESP_OK : ESP_FAIL;
}

/*
 * Put a satellite on the send list as soon as she has an address, rather
 * than at her first probe a quarter-second later. The port is not guessed:
 * satellites bind SYNC_PORT, so it is what client_seen() would have
 * recorded anyway.
 *
 * If the ARP entry cannot be seeded, this registers nothing and lets the
 * probe do it -- degrading to the behaviour that has always worked beats
 * degrading to unicasts that cannot land.
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

    /* The level, at the one moment a unit provably has none. The seeded ARP
     * entry is what makes this first unicast land, and a unit that stays
     * silent until told a level would otherwise sit silent until the 1 Hz
     * repeat reaches her. */
    streamer_send_vol(audio_volume);
    ESP_LOGI(TAG, "satellite " IPSTR " has an address -- on the send list, ARP seeded",
             IP2STR(ip));
}

/*
 * Forget a satellite the instant she disassociates, rather than when she
 * stops probing. The send list is keyed by IP and the WiFi event carries a
 * MAC, so the DHCP lease table bridges them. A failed lookup does nothing
 * and CLIENT_TIMEOUT_US still applies -- this only ever forgets sooner,
 * never something else, because the MAC is authoritative. It earns its
 * keep on the case that actually happens (a reflash or restart, which
 * disassociates cleanly); the timeout covers power loss and walking out
 * of range, which this cannot see at all.
 */
void client_gone(const uint8_t mac[6])
{
    if (!s_ap_netif) {
        return;
    }
    /* Outside the critical section: the lease walk takes its own locks,
     * none of which belong inside a spinlock the send path holds. */
    esp_netif_pair_mac_ip_t pair;
    memcpy(pair.mac, mac, sizeof(pair.mac));
    pair.ip.addr = 0;
    if (esp_netif_dhcps_get_clients_by_mac(s_ap_netif, 1, &pair) != ESP_OK || !pair.ip.addr) {
        /* Counted apart from a real drop: "no lease to resolve" and "nothing
         * on the list to remove" are different facts. The second is the
         * ordinary case for an ungraceful departure -- the AP notices
         * inactivity far later than CLIENT_TIMEOUT_US, so the timeout has
         * already done the work. */
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

    /* Unconditional, not only when the client was still listed: the ARP
     * entry was seeded when the address was assigned and outlives a client
     * the timeout removed first. A failure is not worth a line -- there is
     * nothing to remove if this station never had one seeded. */
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

/*
 * Playback volume to the listeners: addressed to exactly the set the audio
 * goes to, because both walks use the same list -- a unit hears the level
 * exactly when she hears the stream.
 *
 * Repeated, and not redundantly: the level is state, not a stream, so a
 * unit that missed the one packet carrying it stays wrong until something
 * says it again. Three things say it -- streamer_set_volume() sends a
 * change VOL_CHANGE_REPEATS times, client_joined() pushes it at the one
 * moment a unit provably has none, and vol_repeat_start() below repeats it
 * every VOL_REPEAT_US.
 *
 * Silent until audio_vol_known: the local fallback for a bridge that has
 * gone quiet is a playback decision (see audio_vol_effective()), and
 * broadcasting it would take a floor sitting correctly at -50 dB up to
 * full scale.
 */
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

/*
 * Take a new volume from the phone: clamp it, keep it, tell everyone.
 *
 * Clamped rather than trusted -- it arrives from another chip, and a gain
 * built from a byte past full scale would amplify instead of attenuate.
 *
 * A change is sent VOL_CHANGE_REPEATS times: it is the one moment the level
 * is provably wrong everywhere else, and the one packet that carries it can
 * be lost -- the level is state, and nothing else re-states it inside the
 * repeat period. A re-statement sends nothing: the bridge re-states the
 * level on a heartbeat so a rebooted hub recovers, and bursting on those
 * would make n_vol_tx unreadable for the staleness it exists to expose.
 * The first level after boot counts as a change, because
 * audio_vol_known is still false.
 */
void streamer_set_volume(uint8_t volume)
{
    const uint8_t v = volume > AUDIO_VOL_MAX ? AUDIO_VOL_MAX : volume;
    const bool changed = (v != audio_volume) || !audio_vol_known;
    if (changed) {
        ESP_LOGW(TAG, "VOLUME %u/%d", v, AUDIO_VOL_MAX);
    }
    /* Level first, flag second: both are single-byte volatile stores, so a
     * reader that observes the flag necessarily observes the level that
     * goes with it. */
    audio_volume = v;
    audio_vol_known = true;
    if (!changed) {
        return;
    }
    for (int i = 0; i < VOL_CHANGE_REPEATS; i++) {
        streamer_send_vol(v);
    }
}

/**
 * @brief esp_timer callback: re-send the standing level every VOL_REPEAT_US.
 * @param arg  Unused (the timer contract).
 *
 * An esp_timer rather than a counter in the audio path: a sendto in the
 * hub's tightest task buys nothing a timer does not give, and the timer
 * keeps repeating while the stream is stopped -- which is when a satellite
 * is most likely to join unnoticed. Safe in a timer callback because a UDP
 * sendto does not block.
 */
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
/* The batch must fit the datagram the send loop below builds. */
_Static_assert(TX_FRAME_BATCH * sizeof(vis_frame_t) <= FRAME_PAYLOAD_MAX,
               "TX_FRAME_BATCH frames do not fit in frame_msg_t.payload");

/*
 * Take one analysis frame for the listeners and send the batch when it is
 * due. Registered as the visualiser's publisher, so it runs on the analysis
 * task and must not block -- a UDP sendto does not.
 *
 * One datagram per satellite per send: the batch is byte-identical for
 * every listener (analysis computed in one place is what makes that true),
 * so the hub's frame rate grows with the floor but stays N x ~86/s divided
 * by TX_FRAME_BATCH. See TX_FRAME_BATCH in hub.h for what a lost batch
 * costs and what to do if it ever shows.
 */
void publish_frame(const vis_frame_t *f)
{
    /*
     * The batch under construction and the instant it is due out. Static
     * rather than automatic: 400-odd bytes is past what the analysis task's
     * stack should carry, and the whole point is that it survives between
     * calls. Analysis task only, so none of this needs locking.
     */
    static frame_msg_t msg = { .type = MSG_FRAME, .len = (uint8_t)sizeof(vis_frame_t) };
    static uint8_t     pending;
    static int64_t     pending_due;          /* due_us of the newest frame held */
    static int64_t     s_pace_at;

    const int64_t now = esp_timer_get_time();

    /* A batch stranded by a stream that stopped mid-fill: its frames name
     * instants the floor has already passed, and posting them would blink
     * the strips out of time at the start of the next track. Dated by the
     * frames themselves rather than a wall clock, because the analysis runs
     * a lead ahead of playback and a batch delayed by a decoder lump is
     * still perfectly good. */
    if (pending && pending_due > 0 && now > pending_due) {
        n_tx_pace_skip += pending;
        pending = 0;
    }

    memcpy(msg.payload + (size_t)pending * sizeof(*f), f, sizeof(*f));
    pending++;
    pending_due = f->due_us;

    /*
     * Hold until the pace interval is up or the batch is full. The pace
     * decides when this lane may transmit; frames offered in between
     * accumulate rather than drop, and the send carries all of them. The
     * full-batch exit is for the analysis task's lumpiness -- in steady
     * state the pace is what fires. See TX_FRAME_PACE_US in hub.h.
     */
    if (pending < TX_FRAME_BATCH && now < s_pace_at) {
        return;
    }
    s_pace_at = now + TX_FRAME_PACE_US;

    const uint8_t count = pending;
    pending = 0;

    /* Yield the instant the TX pool is exhausted (TX_BACKOFF_US): the audio
     * path is never gated, so this is what keeps a frame burst off the
     * buffers audio is being refused. Counted in frames, not batches, to
     * stay comparable with the 86/s the analysis produces. */
    if (now < s_tx_congested_until) {
        n_tx_cong_skip += count;
        return;
    }
    msg.count = count;
    const size_t bytes = FRAME_MSG_BYTES((size_t)count * sizeof(*f));

    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    /* Stamped before the loop, not after: the buffers are held from the
     * first sendto onwards, so dating the batch from when it finished would
     * miss exactly the overlap n_refuse_near_frame exists to catch. */
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
