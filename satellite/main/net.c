/*
 * Joining the hub's SoftAP, and the one UDP socket everything rides on.
 *
 * Split out of main.c on 2026-08-12; the bodies are unchanged.
 */
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"

#include "audio_out.h"
#include "sync_proto.h"
#include "sbc_link.h"
#include "sbc_decoder.h"
#include "visualiser.h"
#include "wifi_log.h"

#include "sat.h"

/* ------------------------------------------------------------------- wifi */

/*
 * How long to wait before trying the association again.
 *
 * Unchanged from the vTaskDelay this replaces -- the backoff was never the
 * problem, only where it was being taken. See wifi_retry_tick().
 */
#define WIFI_RETRY_BACKOFF_US 1000000

/*
 * How long an association may sit there without producing a DHCP lease.
 *
 * The two are separate events and only the second one puts this unit on the
 * air, so "associated" is not a state to wait in indefinitely -- see the lease
 * branch of wifi_retry_tick() for the hole that leaves.
 *
 * Ten seconds is chosen against lwIP's own behaviour rather than against
 * patience: the DHCP client retransmits its discover on a doubling backoff, so
 * this is roughly three attempts. Shorter would tear down associations that
 * were about to work on a floor where the hub is briefly busy; much longer and
 * a unit that will never get a lease sits dark for no reason.
 */
#define WIFI_LEASE_TIMEOUT_US 10000000

/*
 * The reconnect state, owned by the event handler and read by the probe task.
 *
 * THREE STATES, NOT TWO, because association and lease are separate events and
 * the failure modes either side of the join are different things:
 *
 *   !s_assoc               not on the AP          -> ask to connect
 *   s_assoc && !s_link_up  on the AP, no address  -> wait, then tear it down
 *   s_link_up              on the floor           -> nothing to do
 *
 * s_link_up is a DHCP lease, not an association, for the same reason the
 * marker's link level is: a lease is the point at which this unit can actually
 * be sent audio.
 *
 * s_retry_at and s_lease_at are the local instants their state's next action is
 * due, 0 when none is. Both are only ever set FORWARD, and never cleared by the
 * tick that fires on one: see wifi_retry_tick() for the races that costs
 * nothing and the one it would cost the unit.
 *
 * volatile, not atomic. All four are written on the event loop task and read on
 * the probe task, and none can tear -- the bools trivially, the int64s because
 * a stale read is one 250 ms tick of latency and nothing worse.
 */
static volatile bool    s_assoc;
static volatile bool    s_link_up;
static volatile int64_t s_retry_at;
static volatile int64_t s_lease_at;

/*
 * Ask for the association again when the backoff is up. Called once per probe,
 * so within PROBE_PERIOD_MS of the instant it is due.
 *
 * THIS USED TO BE A vTaskDelay INSIDE THE EVENT HANDLER, which runs on the
 * default event loop's task -- so a disconnect stopped esp_event dead for a
 * whole second, and a streak of them stopped it for a second each. Everything
 * the loop carries was stuck behind it, GOT_IP included: the lease that ends
 * the outage was queued behind the sleep taken because the outage had started.
 * Nothing here is worth a second of that, and the handler now returns
 * immediately.
 *
 * Driven by the flag rather than by a one-shot, so the retry does not depend on
 * every failed attempt raising WIFI_EVENT_STA_DISCONNECTED to schedule the
 * next. If one is ever missed the deadline below still fires; the old chain
 * would have stopped at that point with the unit off the air and nothing left
 * to restart it.
 *
 * Here rather than in a task of its own. A satellite that cannot probe cannot
 * anchor and has no audio, so folding the retry into that task adds no failure
 * this unit could otherwise survive -- and 250 ms is the whole granularity cost
 * against a 1 s backoff.
 */
void wifi_retry_tick(void)
{
    if (s_link_up) {
        return;
    }
    const int64_t now = esp_timer_get_time();

    /*
     * ASSOCIATED, AND NO ADDRESS. The other half of the same fault.
     *
     * An association that succeeds raises STA_CONNECTED and nothing else. If
     * DHCP then never completes there is no lease, and -- this is the part that
     * strands the unit -- no STA_DISCONNECTED either, because as far as the
     * radio is concerned everything is fine. So the retry below is never armed,
     * this unit sits on the AP with no address, its probes go nowhere, the hub
     * never hears of it, and nothing in the firmware is unhappy enough to try
     * anything. Only a reboot ended it.
     *
     * The way out is to stop being associated. esp_wifi_disconnect() raises
     * STA_DISCONNECTED, which is the event the handler already knows what to do
     * with, so this drops into the reconnect path below rather than inventing a
     * second one -- and the association that comes back runs DHCP again from
     * the start, which is the thing that actually needs re-doing.
     *
     * Note this branch is also what stops the reconnect below from firing at an
     * association that is merely still waiting for its lease. Without the state
     * split it would call esp_wifi_connect() once a second at a link that is
     * halfway up.
     */
    if (s_assoc) {
        const int64_t due = s_lease_at;
        if (due == 0 || now < due) {
            return;
        }
        s_lease_at = now + WIFI_LEASE_TIMEOUT_US;
        /*
         * Read again, immediately before acting on a deadline ten seconds old.
         * A lease landing in the window between the test above and the call
         * below would otherwise have its link torn down by the watchdog that
         * was waiting for it -- recoverable, since the reconnect path picks it
         * up a second later, but a self-inflicted outage either way. One load
         * to make the window nil.
         */
        if (s_link_up) {
            return;
        }
        n_wifi_lease_fail++;
        ESP_LOGE(TAG, "associated to \"%s\" for %d ms with no DHCP lease -- "
                      "dropping the association to start over",
                 AP_SSID, WIFI_LEASE_TIMEOUT_US / 1000);
        esp_wifi_disconnect();
        return;
    }

    const int64_t due = s_retry_at;
    if (due == 0 || now < due) {
        return;
    }
    /*
     * Armed again BEFORE the attempt, not cleared by it.
     *
     * Clearing would open the race that strands the unit: a disconnect landing
     * between the read above and the clear would arm a deadline this then wipes,
     * and with no further event coming nothing would ever ask again. Pushing it
     * forward instead means the two races both cost nothing -- a disconnect
     * arriving now simply re-dates an attempt that is already due, and an
     * association arriving now raises s_assoc, so the next tick takes the
     * branch above and this deadline is simply never read.
     */
    s_retry_at = now + WIFI_RETRY_BACKOFF_US;
    esp_wifi_connect();
}

/*
 * Without this a failed association is completely silent: esp_wifi_connect() is
 * called once, and if it does not succeed nothing logs it and nothing retries.
 * A satellite that quietly never joins is far worse than one that says so.
 */
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        /* Armed as well as attempted. If this first call fails outright -- it
         * returns an error rather than raising STA_DISCONNECTED -- there is no
         * event coming to arm the retry, and the unit would never ask again. */
        s_retry_at = esp_timer_get_time() + WIFI_RETRY_BACKOFF_US;
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        /*
         * On the AP, not yet on the floor. Start the clock on the lease.
         *
         * Nothing else is said here: an association is not news worth a line on
         * its own, and the join is already logged where it means something --
         * at GOT_IP below, which is the event that ends the outage. What this
         * arms is the watchdog for the case where that event never comes.
         */
        s_assoc = true;
        s_lease_at = esp_timer_get_time() + WIFI_LEASE_TIMEOUT_US;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = data;
        n_wifi_drops++;
        /* Both flags down before the deadline is armed, so a tick that sees the
         * deadline cannot also see a stale state and take the wrong branch on
         * it. The lease deadline goes with the association it was measuring. */
        s_assoc = false;
        s_link_up = false;
        s_lease_at = 0;
        s_retry_at = esp_timer_get_time() + WIFI_RETRY_BACKOFF_US;
        /* Only the first drop of a streak dates the outage: repeated
         * association failures raise this repeatedly, and taking the latest
         * would measure the last retry rather than how long the unit was off
         * the air, which is the number that matters to playback. */
        if (wifi_down_at == 0) {
            wifi_down_at = esp_timer_get_time();
        }
        /*
         * Off the floor, so the marker LED stops claiming otherwise.
         *
         * Said on every drop of a streak rather than only the first, unlike
         * wifi_down_at above: that one is dating an outage and must not be
         * restarted by a retry, while this is asserting a level and repeating
         * it costs nothing.
         *
         * Nothing here waits for playback to notice. Audio outlives a drop by
         * roughly the ring's depth, so the LED goes dark a few hundred ms
         * before the flashes stop -- and it is the flashes that stop late, not
         * this that goes early. During those few hundred ms the render task is
         * still drawing, so the marker is still flashing and this level is not
         * shown until it falls idle. That is deliberate: audio still flowing IS
         * the link working, whatever the association says.
         */
        visualiser_marker_set_link(false);
        ESP_LOGW(TAG, "disconnected from \"%s\" (reason %d), retrying in %d ms",
                 AP_SSID, d->reason, WIFI_RETRY_BACKOFF_US / 1000);
        /* And that is the whole handler. The attempt itself belongs to
         * wifi_retry_tick() on the probe task -- see it for why waiting for the
         * backoff here was costing far more than the retry it bought. */
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        /* Stops the retry, and disarms the watchdog that was waiting for
         * exactly this. A lease, not an association -- the same line
         * visualiser_marker_set_link(true) below is drawn on, and for the same
         * reason: this is the point at which the unit can be sent audio. */
        s_link_up = true;
        s_lease_at = 0;
        if (wifi_down_at) {
            /*
             * How long this unit was actually off the air. Playback survives
             * roughly the ring's worth of it and then underruns, so this is the
             * number that says whether a drop cost a glitch or a re-anchor --
             * and it is the baseline any change to the reconnect path has to
             * beat. rejoined_at arms the report on the next anchor.
             */
            ESP_LOGW(TAG, "rejoined \"%s\" after %lld ms",
                     AP_SSID, (esp_timer_get_time() - wifi_down_at) / 1000);
            wifi_down_at = 0;
            rejoined_at = esp_timer_get_time();
        }
        /*
         * On the floor. The marker LED goes solid until audio starts, which is
         * the first thing this unit can say from across a dark field without a
         * console -- and the thing it could not say before, when a satellite
         * that never found the hub and a satellite waiting for music were both
         * simply dark.
         *
         * Here rather than at association, because a lease is the point at
         * which the unit can actually be sent audio. It also repeats on every
         * rejoin, which is what re-lights the LED after a drop.
         *
         * Safe this early: the setter only stores a flag, and the render task
         * picks it up whenever it starts -- visualiser_start() runs after
         * wifi_start_sta() and a fast lease can beat it here.
         */
        visualiser_marker_set_link(true);
        ESP_LOGI(TAG, "joined \"%s\", IP " IPSTR, AP_SSID, IP2STR(&e->ip_info.ip));
        /*
         * Said here, not where it happened. task_start() logs the failure at
         * boot, roughly a second before this unit has a route, so wifi_log drops
         * that line for want of a destination and the collector never sees it.
         * This is the first moment the radio can carry the news.
         *
         * Silent on a healthy unit, like every other counter here. If it ever
         * prints, nothing else in this log means what it usually means.
         *
         * Ordering is assumed, not enforced: wifi_start_sta() runs about a
         * second before the tasks are created, so a DHCP lease arriving that
         * fast would find n_task_fail still zero and say nothing. The console
         * line in task_start() is unconditional, and this one repeats on every
         * rejoin, so the news is only lost if both the console is dead AND the
         * unit never reconnects. Worth knowing before trusting a silent join.
         */
        if (n_task_fail) {
            ESP_LOGE(TAG, "CRIPPLED: %" PRIu32 " task(s) failed to start: %s",
                     n_task_fail, s_task_fail_names);
        }
    }
}

void wifi_start_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wc = {0};
    strcpy((char *)wc.sta.ssid, AP_SSID);
#if CONFIG_DANCEFLOOR_AP_OPEN
    wc.sta.password[0] = '\0';
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
#else
    strcpy((char *)wc.sta.password, AP_PASS);
#endif
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                       wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                       wifi_event, NULL, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

#if CONFIG_DANCEFLOOR_DISABLE_PMF
    /*
     * Both ends, so neither advertises the capability -- see the hub's copy for
     * what PMF was doing to this link. This unit is the one that was failing to
     * answer the SA Query and being thrown off for it, and it is also the one
     * that never noticed: the hub counted two disassociations while this
     * counted zero.
     *
     * Must sit between esp_wifi_set_config() and esp_wifi_start().
     */
    const esp_err_t pmf = esp_wifi_disable_pmf_config(WIFI_IF_STA);
    ESP_LOGW(TAG, "PMF disabled on the station: %s", esp_err_to_name(pmf));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    /* STA_START triggers the first connect; disconnects retry from the handler. */
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
    /* Nothing else to do: audio arrives by unicast, and the time probes this
     * unit sends are what put it on the hub's send list. */
}
