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
    /* The level, at the one moment a unit provably has none. The ARP entry was
     * seeded three lines up, which is the condition that makes a first unicast
     * land -- the whole subject of this function's comment -- so this is a second
     * and natural user of it. Under multicast it goes to the group like every
     * other copy; either way it closes the join window from "up to one repeat
     * interval" to one packet time, and with a unit that stays silent until told,
     * that window is silence rather than full scale. */
    streamer_send_vol(audio_volume);
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

/*
 * Playback volume, hub -> listeners.
 *
 * ADDRESSED TO THE SAME SET AS THE AUDIO, BY CONSTRUCTION. That is the whole
 * point of the #if below, and it is what this got wrong for as long as the audio
 * has been multicast: fan_out() addresses a GROUP and this addressed the CLIENT
 * LIST, and those are different sets. The difference is exactly the set of units
 * that play audio at a level nobody told them -- anything that clears a slot
 * (CLIENT_TIMEOUT_US of no probe, a disassociation, the gap between
 * re-associating and the first probe) leaves a unit still hearing the stream and
 * no longer hearing the level. Under multicast, hearing the audio and hearing
 * the level are now the same condition.
 *
 * The satellite needs nothing for this. net.c joins the group on the same socket
 * it binds INADDR_ANY:SYNC_PORT on, and rx_task demuxes by type from one
 * recvfrom, so a group-addressed MSG_VOL lands in the branch that already
 * existed.
 *
 * WHAT A GROUP FRAME COSTS, AND HOW IT IS PAID FOR. It is never acknowledged and
 * never retried, where a unicast has a link-layer ACK. For two bytes that is
 * cheap to buy back with repetition, and there are three repetitions:
 * streamer_set_volume() sends the change three times, client_joined() pushes it
 * at the one moment a unit provably has none, and vol_repeat_start() below
 * repeats it every second. Against the measured 0.2-0.3% group loss, exposure to
 * a stale level is P(loss) x 1 s, and only in the window after a change.
 *
 * The old comment here argued for unicast so the level "must not be held for a
 * DTIM burst behind the audio it would delay". That was never true on this
 * build: hub and satellite both set WIFI_PS_NONE, so nothing is buffered for a
 * DTIM at all -- net.c says so where it sets dtim_period.
 *
 * The unicast arm is kept for CONFIG_DANCEFLOOR_AUDIO_MCAST=n, where there is no
 * group, the satellite never joins one, and the client list IS the audio set.
 * That build is unchanged.
 */
void streamer_send_vol(uint8_t volume)
{
    /* Never relay a level nobody gave us. The hub has its own fallback for a
     * bridge that has gone quiet, and broadcasting it would take a floor sitting
     * correctly at -50 dB up to full scale -- so the fallback stays a local
     * playback decision and this stays silent until there is a real level to
     * repeat. See audio_vol_effective(). */
    if (sock < 0 || !audio_vol_known) {
        return;
    }
    vol_msg_t msg = { .type = MSG_VOL, .volume = volume };

#if CONFIG_DANCEFLOOR_AUDIO_MCAST
    /* flags=0, unlike the audio's MSG_DONTWAIT. This runs on the timer task or
     * an event handler, never on the SBC receive task that also feeds the hub's
     * own ring, so there is no local feed here for a blocking send to starve --
     * and a two-byte frame that waits for a TX slot is the outcome we want over
     * one that is dropped. */
    if (sendto(sock, &msg, sizeof(msg), 0,
               (const struct sockaddr *)mcast_addr(), sizeof(struct sockaddr_in)) >= 0) {
        n_vol_tx++;
    }
#else
    client_t snapshot[MAX_CLIENTS];
    clients_snapshot(snapshot);

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (snapshot[i].last_seen) {
            sendto(sock, &msg, sizeof(msg), 0,
                   (struct sockaddr *)&snapshot[i].addr, sizeof(snapshot[i].addr));
            n_vol_tx++;
        }
    }
#endif
}

/*
 * Take a new volume from the phone: clamp it, keep it, and tell everyone.
 *
 * Clamped rather than trusted -- it arrives from another chip, and a gain built
 * from a byte past full scale would amplify instead of attenuate.
 *
 * A CHANGE IS SENT THREE TIMES; A RE-STATEMENT IS NOT SENT AT ALL. A change is
 * the one moment the level is provably wrong everywhere else, and under
 * multicast there is no link-layer retry to lean on -- four bytes of extra air
 * against a slider move a human is watching the result of, where a single lost
 * frame would otherwise be audible until the next second's repeat, which at the
 * bottom of the taper is a large step.
 *
 * But most calls here are NOT changes. The bridge re-states the level on a 5 s
 * heartbeat so a rebooted hub recovers, and every one of those arrives through
 * this function. Bursting on them made the hub send 1.6 levels a second instead
 * of one -- 96 per minute in logs-soak-20260818-150804, which is 60 from the
 * repeat plus twelve heartbeats' worth of threes. Harmless in airtime, and wrong
 * in two ways that matter more: the burst stopped meaning "something changed",
 * and n_vol_tx stopped being readable for the staleness it exists to expose.
 *
 * So a re-statement sends nothing. The heartbeat's job is to tell THIS unit; the
 * satellites are served by the 1 Hz repeat either way, and the first level after
 * a boot is a change by this test because audio_vol_known is still false.
 */
void streamer_set_volume(uint8_t volume)
{
    const uint8_t v = volume > AUDIO_VOL_MAX ? AUDIO_VOL_MAX : volume;
    const bool changed = (v != audio_volume) || !audio_vol_known;
    if (changed) {
        ESP_LOGW(TAG, "VOLUME %u/%d", v, AUDIO_VOL_MAX);
    }
    /* Level first, flag second, for the reason sat.h gives. */
    audio_volume = v;
    audio_vol_known = true;
    if (!changed) {
        return;
    }
    for (int i = 0; i < VOL_CHANGE_REPEATS; i++) {
        streamer_send_vol(v);
    }
}

/*
 * Repeat the level once a second, for the whole life of the hub.
 *
 * This replaces the repeat that used to sit in telemetry_tick(), and the
 * interval matters more than it did: with a unit that stays silent until it is
 * told a level, this interval IS the silence a joining satellite sits through if
 * client_joined()'s push is lost. Five seconds of silence was a fine price when
 * the alternative was five seconds at full scale; one second is a better one.
 *
 * Its own esp_timer rather than a counter in fan_out(). Driving it from the
 * audio would tie the repeat to the stream neatly, but it would put a sendto in
 * the hub's tightest task -- decode, two stream buffers and a send in one block,
 * priority 9, the thing dma_starve exists to catch -- for no gain a timer does
 * not already give. A timer also keeps repeating while the stream is stopped,
 * which is when a satellite is most likely to join unnoticed.
 *
 * Safe in a timer callback: sendto() on a UDP socket does not block, which is
 * the same argument publish_frame() makes for running on the analysis task.
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
/*
 * Send one analysis frame to every listener.
 *
 * Registered as the visualiser's publisher, so it runs on the analysis task --
 * which means it must not block. sendto() on a UDP socket does not.
 *
 * One group send, or one per satellite -- see DANCEFLOOR_MCAST_FRAMES for why
 * the group is now the default and why the "~20% loss" objection that kept this
 * unicast was a measurement of the 1 Mbps basic rate rather than of multicast.
 * The frame is identical for every listener either way, which is what makes the
 * choice a pure transport question.
 *
 * A failed send costs a satellite one frame out of 86 a second. It is counted
 * with the audio's own failures rather than separately -- the interesting
 * question is whether the link is dropping things, not which kind.
 */
void publish_frame(const vis_frame_t *f)
{
    /* PROACTIVE first, reactive second -- the order is the point.
     *
     * The pace gate stops a burst from forming at all: no two frame sends
     * closer together than TX_FRAME_PACE_US, which in steady state (frames
     * arriving on the analysis cadence) never binds, and during a decoder
     * lump staggers the frames a quarter-period apart instead of letting them
     * crowd the pool the audio shares. Analysis task only -- s_pace_at needs
     * no locking for the same reason the congestion gate does not.
     *
     * The backoff gate below then handles the failure the pace cannot: a pool
     * exhausted by something else (audio's own burst, probes) still yields
     * the lane until it recovers. */
    static int64_t s_pace_at;
    const int64_t now = esp_timer_get_time();
    if (now < s_pace_at) {
        n_tx_pace_skip++;
        return;
    }
    s_pace_at = now + TX_FRAME_PACE_US;

    /* Yield the instant the TX pool is exhausted: see TX_BACKOFF_US. fan_out() --
     * the audio path -- is never gated, so this is what keeps a frame burst off the
     * buffers audio is being refused. */
    if (now < s_tx_congested_until) {
        n_tx_cong_skip++;
        return;
    }
    if (sizeof(*f) > FRAME_PAYLOAD_MAX) {
        return;                              /* refuse rather than truncate */
    }
    frame_msg_t msg = { .type = MSG_FRAME, .len = (uint8_t)sizeof(*f) };
    memcpy(msg.payload, f, sizeof(*f));
    const size_t bytes = FRAME_MSG_BYTES(sizeof(*f));

#if CONFIG_DANCEFLOOR_MCAST_FRAMES
    /* Blocking (flags=0), unlike the audio group send. This runs on the analysis
     * task, which is below playback and feeds nothing on the audio path, so a
     * moment spent waiting for a transmit buffer costs a late frame rather than
     * a hole in the sound -- and waiting is better than the drop that
     * MSG_DONTWAIT would make of it. The audio path cannot make that trade
     * because it shares a task with the hub's own ring feed. */
    if (sendto(sock, &msg, bytes, 0,
               (const struct sockaddr *)mcast_addr(), sizeof(struct sockaddr_in)) < 0) {
        tx_fail_note(errno);
    }
#else
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
#endif
}
#endif

