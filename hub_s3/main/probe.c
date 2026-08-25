
#include "hub.h"

void probe_task(void *arg)
{
    (void)arg;

    uint8_t buf[256];
    struct sockaddr_in from;

    while (1) {

        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        int64_t t2 = esp_timer_get_time();

#if CONFIG_DANCEFLOOR_WIFI_LOGS

        if (n >= (int)sizeof(log_sub_msg_t) && buf[0] == MSG_LOG_SUB) {
            log_sub_msg_t sub;
            memcpy(&sub, buf, sizeof sub);
            if (sub.magic == LOG_SUB_MAGIC) {
                wifi_log_note_collector(from.sin_addr.s_addr);
            }
            continue;
        }

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

        if (n >= (int)sizeof(splice_msg_t) && buf[0] == MSG_SPLICE) {
            splice_msg_t s;
            memcpy(&s, buf, sizeof(s));
            client_seen(&from);
            const int64_t age = s_hub_splice_at ? (t2 - s_hub_splice_at) / 1000 : -1;
            const char *who = inet_ntoa(from.sin_addr);
            if (age >= 0 && age < 10000) {

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

                ESP_LOGW(TAG, "satellite %s spliced %+ld ms (phase %+ld us), "
                              "no hub boundary to compare",
                         who, (long)(s.applied_us / 1000), (long)s.phase_us);
            }
            continue;
        }

        if (n < (int)sizeof(time_msg_t) || buf[0] != MSG_TIME_REQ) {
            continue;
        }
        client_seen(&from);

        time_msg_t msg;
        memcpy(&msg, buf, sizeof(msg));
        msg.type = MSG_TIME_RSP;
        msg.t2 = t2;
        msg.t3 = esp_timer_get_time();

        if (t2 >= s_tx_congested_until &&
            sendto(sock, &msg, sizeof(msg), 0,
                   (struct sockaddr *)&from, from_len) < 0) {
            tx_fail_note(TX_LANE_PROBE, errno);
        }

        const int64_t tsf = esp_wifi_get_tsf_time(WIFI_IF_AP);
        const int64_t now = esp_timer_get_time();

        static bool told;
        if (!told) {
            told = true;
            if (tsf == 0) {

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
