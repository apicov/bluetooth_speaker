/*
 * The time probe server, and everything else that arrives on the sync socket.
 *
 * A satellite's clock offset is a round trip against this task: it stamps t2 on
 * arrival and t3 immediately before the reply, so the satellite can take out the
 * transit time. That is the whole clock distribution mechanism.
 *
 * The same socket also carries satellite splice reports (the cross-unit
 * divergence measurement that works without a marker wire) and, on bench
 * builds, the log/health relay to a collector.
 */
#include "hub.h"

void probe_task(void *arg)
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
                 * The second clause is the counterfactual, and the roles have
                 * SWAPPED. Both units now splice on the median of their last few
                 * phase readings; `raw:` is what `apart` would have been on the
                 * newest reading alone, which is what they used to do. Both
                 * figures are measured at the same boundary from the same
                 * histories, so they compare directly -- unlike two builds' log
                 * windows, which cannot, and which is how three wrong diagnoses
                 * were reached here. If `raw:` is consistently the SMALLER one
                 * across a session, this change was wrong and should be reverted.
                 */
                ESP_LOGW(TAG, "TRACK DIVERGENCE (wifi): %s spliced %+ld ms "
                              "(phase %+ld us), hub spliced %+ld ms -> %+ld ms apart"
                              " | raw: sat %+ld ms, hub %+ld ms -> %+ld ms apart",
                         who, (long)(s.applied_us / 1000), (long)s.phase_us,
                         (long)(s_hub_splice_us / 1000),
                         (long)((s.applied_us - s_hub_splice_us) / 1000),
                         (long)(s.applied_alt_us / 1000),
                         (long)(s_hub_splice_alt_us / 1000),
                         (long)((s.applied_alt_us - s_hub_splice_alt_us) / 1000));
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
        /*
         * The reply yields to the ENOMEM backoff, because it is non-audio and
         * that is the contract: s_tx_congested_until in hub.h says non-audio
         * publishing stands down so the buffers audio is being refused are left
         * for audio, and publish_frame has honoured it since it was written.
         * This lane did not, and the 2026-08-22 soak's worst window shows it --
         * `tx-fail 251 (213 audio, 4 vol, 34 probe)` with the frame lane already
         * fully suppressed at cong-skip 191.
         *
         * A refused sendto costs no buffer, so this frees nothing directly; it
         * stops probe winning the race for buffers as they come back, which at
         * ~16 probe sends/s against audio's ~100 is a modest share and is not
         * expected to fix anything on its own.
         *
         * Skipping a reply is close to free on its own terms. A round trip
         * measured through a congested link carries inflated and asymmetric
         * transit, which is precisely the error the estimator cannot see and
         * cannot remove, so the sample withheld here is one it is better off
         * not having. The satellite asks again in 250 ms.
         *
         * Not counted on the status line. cong-skip is documented as a count of
         * FRAMES, kept comparable with the 86/s the analysis produces, and
         * mixing probe datagrams into it would break that; a suppressed reply
         * is visible anyway as the probe lane's share of tx-fail going to zero.
         */
        if (t2 >= s_tx_congested_until &&
            sendto(sock, &msg, sizeof(msg), 0,
                   (struct sockaddr *)&from, from_len) < 0) {
            tx_fail_note(TX_LANE_PROBE, errno);
        }
        /* Either way execution continues to the TSF message below, which is
         * deliberately not gated -- see the reason where it is sent. */

        /*
         * Measurement only -- see tsf_msg_t. Sent on the back of the probe
         * reply because that already runs once per satellite per probe and
         * needs no task, no client snapshot and no timer of its own. 17 bytes
         * four times a second against ~42 kB/s of audio.
         *
         * The pair is read as close together as it can be: the gap between them
         * is skew that lands directly in the comparison.
         *
         * NOT GATED BY THE ENOMEM BACKOFF, unlike the reply just above, and the
         * asymmetry is deliberate.
         *
         * This is the satellite's PRIMARY clock source -- clock_offset() in
         * satellite/main/clock.c prefers it and keeps the probe estimator only
         * as a fallback -- and it goes stale at TSF_MAX_AGE_US, one second.
         *
         * This once read that the worst refusal stretch measured was 909 ms, so
         * gating would leave the satellite at the edge of that cliff without
         * pushing it over. THAT IS NO LONGER TRUE and the reasoning has to be
         * stated the other way round. The 3h53m soak of 2026-08-22
         * (logs-soak-20260822-165644) recorded four stretches past the second --
         * 1003, 1284, 1298 and 1313 ms -- and sat_s3 fell onto the estimator
         * twice during that episode, at +11523 s and +11580 s, with this
         * message ungated the whole time.
         *
         * So the cliff gets crossed either way, and gating buys nothing against
         * it. What gating would still cost is every TSF message in the stretches
         * that stay under a second, which is most of them: it would remove
         * samples that were doing their job without preventing the staleness
         * that matters. That is why it stays ungated -- not because the
         * satellite is safely inside the window.
         *
         * It is also the one message congestion does not corrupt: there is no
         * round trip in it, so it carries none of the path asymmetry (clock.c
         * says so where it chooses between the two). 17 bytes at 4/s is not
         * what exhausts a 38-buffer pool.
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
        if (sendto(sock, &tm, sizeof(tm), 0,
                   (struct sockaddr *)&from, from_len) < 0) {
            tx_fail_note(TX_LANE_PROBE, errno);
        }
    }
}
