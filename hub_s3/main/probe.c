/**
 * @file probe.c
 * @brief The time-probe server, and everything else on the sync socket.
 *
 * A satellite's clock offset is a round trip against this task: t2 stamped
 * on arrival, t3 immediately before the reply, so the satellite can subtract
 * its own transit time. That is the whole clock distribution mechanism. The
 * same socket also carries satellite splice reports (the cross-unit
 * divergence measurement that needs no marker wire) and, on WIFI_LOGS
 * builds, the log and health relay to a collector.
 */
#include "hub.h"

void probe_task(void *arg)
{
    (void)arg;
    /* Sized for the largest thing that can arrive: a log_msg_t, ~222 bytes.
     * The time and splice messages are far smaller. */
    uint8_t buf[256];
    struct sockaddr_in from;

    while (1) {
        /* recvfrom writes the real address length back, so this resets every
         * iteration rather than hoisting out of the loop. */
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        int64_t t2 = esp_timer_get_time();          /* stamp on arrival */

#if CONFIG_DANCEFLOOR_WIFI_LOGS
        /* Collector registration. Deliberately not client_seen(): a laptop
         * never sends a time probe, so it must not enter s_clients and be
         * unicasted audio like a satellite. Refreshed on every SUB; the
         * relay ages the address out if the collector stops sending. */
        if (n >= (int)sizeof(log_sub_msg_t) && buf[0] == MSG_LOG_SUB) {
            log_sub_msg_t sub;
            memcpy(&sub, buf, sizeof sub);
            if (sub.magic == LOG_SUB_MAGIC) {
                wifi_log_note_collector(from.sin_addr.s_addr);
            }
            continue;
        }

        /* A satellite's log line or health snapshot, relayed verbatim. The
         * source address is stamped here because the satellite does not know
         * its own DHCP lease. Not client_seen(): a satellite registers by its
         * probes anyway, and the relay is two non-blocking sendto()s that
         * cannot hold up the probe path. */
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
         * units splice by their own error against the same published
         * timeline, so the DIFFERENCE between the two corrections is how far
         * apart they had drifted over that track -- the marker wire's
         * question, answered over the WiFi that is there anyway.
         */
        if (n >= (int)sizeof(splice_msg_t) && buf[0] == MSG_SPLICE) {
            splice_msg_t s;
            memcpy(&s, buf, sizeof(s));
            client_seen(&from);
            const int64_t age = s_hub_splice_at ? (t2 - s_hub_splice_at) / 1000 : -1;
            const char *who = inet_ntoa(from.sin_addr);
            if (age >= 0 && age < 10000) {
                /* `raw:` is the counterfactual: what `apart` would have been
                 * had both units spliced on their newest phase reading alone
                 * rather than the median. Both figures come from the same
                 * boundary and the same histories, so they compare directly;
                 * if `raw` is consistently the smaller across a session, the
                 * median was the wrong choice. */
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
                /* No hub boundary to pair with: the hub's phase was invalid,
                 * or this arrived nowhere near one. */
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
         * The reply honours the ENOMEM backoff, like every non-audio lane:
         * s_tx_congested_until stands down exactly the lanes whose buffers
         * audio competes for. A refused sendto frees no buffer directly; what
         * it stops is the probe lane winning buffers back as they free. And
         * a round trip measured through congestion carries inflated,
         * asymmetric transit -- the one error the estimator cannot remove --
         * so the withheld sample is one it is better off without. The
         * satellite asks again within PROBE_PERIOD_MS.
         *
         * Not counted anywhere: cong-skip counts frames, to stay comparable
         * with the 86/s the analysis produces, and a suppressed reply is
         * visible anyway as the probe lane's share of tx-fail going to zero.
         */
        if (t2 >= s_tx_congested_until &&
            sendto(sock, &msg, sizeof(msg), 0,
                   (struct sockaddr *)&from, from_len) < 0) {
            tx_fail_note(TX_LANE_PROBE, errno);
        }

        /*
         * The TSF pair -- measurement only, see tsf_msg_t. Ridden on the
         * probe reply because that already runs once per satellite per probe
         * and needs no task or timer of its own: 17 bytes a few times a
         * second. Read back to back, since the gap between the two readings
         * is skew that lands directly in the comparison.
         *
         * NOT gated by the backoff, unlike the reply above, and the
         * asymmetry is deliberate. This is the satellite's primary clock
         * source (clock.c prefers it, keeping the probe estimator as
         * fallback) and it goes stale at TSF_MAX_AGE_US, one second: gating
         * would discard good samples in every congested stretch shorter than
         * that without preventing the staleness that actually costs. And
         * congestion cannot corrupt it -- there is no round trip in it, so
         * it carries none of the transit asymmetry the estimator suffers.
         */
        const int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_AP);
        const int64_t now = esp_timer_get_time();
        /* Say which way it went, once: a run with no TSF output should not
         * be ambiguous between "not supported" and "wrong branch". */
        static bool told;
        if (!told) {
            told = true;
            if (tsf == 0) {
                /* A persistent zero is a result: SoftAP-side TSF is not
                 * exposed on this target, and the experiment stops. */
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
