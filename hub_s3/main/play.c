
#include "hub.h"
#include "audio_shift.h"

static int32_t s_samples_played;
static int64_t s_wrote_at;

static uint8_t chunk[AUDIO_CHUNK_BYTES];

static uint32_t s_play_epoch;

typedef enum {
    CHUNK_OK,
    CHUNK_UNDERRUN,
} chunk_result_t;

static bool park_for_retune(void)
{
    if (retuning) {

        vTaskDelay(pdMS_TO_TICKS(2));
        return true;
    }

    return false;
}

#define TRIM_ONE_FRAME 65536
static int32_t s_trim_owed;

static int trim_due(void)
{
    const int32_t hz = rate_trim_hz;
    if (hz == 0) {

        s_trim_owed = 0;
        return 0;
    }
    s_trim_owed += (int32_t)((int64_t)AUDIO_FRAMES * hz * TRIM_ONE_FRAME
                             / (int64_t)tx_rate);

    if (s_trim_owed >  2 * TRIM_ONE_FRAME) s_trim_owed =  2 * TRIM_ONE_FRAME;
    if (s_trim_owed < -2 * TRIM_ONE_FRAME) s_trim_owed = -2 * TRIM_ONE_FRAME;
    if (s_trim_owed >= TRIM_ONE_FRAME) {
        s_trim_owed -= TRIM_ONE_FRAME;
        return 1;
    }
    if (s_trim_owed <= -TRIM_ONE_FRAME) {
        s_trim_owed += TRIM_ONE_FRAME;
        return -1;
    }
    return 0;
}

static int16_t s_cu_in[(AUDIO_FRAMES + CATCHUP_SHIFT_MAX + 1) * AUDIO_CHANNELS];
static int s_catchup_take;
static int32_t s_catchup_moved;

static int chunk_shift(void)
{
    s_catchup_take = 0;
    const int trim = trim_due();

    const int32_t debt = catchup_frames;
    if (debt == 0) {
        return trim;
    }

    const int32_t level_ms = (int32_t)((int64_t)(LOCAL_RING_BYTES -
            (int64_t)xStreamBufferSpacesAvailable(local_ring)) * 1000
            / ((int64_t)sample_rate * AUDIO_CHANNELS * 2));
    if (debt > 0 && level_ms <= (int32_t)(LEAD_US / 1000) - 30) {
        return trim;
    }
    if (debt < 0 && level_ms >= (int32_t)(LEAD_US / 1000) + 50) {
        return trim;
    }

    int32_t k = debt;
    if (k >  CATCHUP_SHIFT_MAX_DROP) k =  CATCHUP_SHIFT_MAX_DROP;
    if (k < -CATCHUP_SHIFT_MAX_DUP)  k = -CATCHUP_SHIFT_MAX_DUP;
    s_catchup_take = (int)k;
    return (int)k + trim;
}

static chunk_result_t read_chunk(uint32_t *got_frames)
{

    const int shift = chunk_shift();
    const bool cu = (shift > 1 || shift < -1);
    const size_t frame_bytes = AUDIO_CHANNELS * sizeof(int16_t);
    const size_t want = cu
        ? (size_t)((int)AUDIO_FRAMES + shift) * frame_bytes
        : (shift < 0) ? sizeof(chunk) - frame_bytes : sizeof(chunk);

    hw_play = uxTaskGetStackHighWaterMark(NULL);
    uint8_t *const dest = cu ? (uint8_t *)s_cu_in : chunk;
    size_t got = xStreamBufferReceive(local_ring, dest, want, pdMS_TO_TICKS(500));
    if (got == 0) {
        n_underruns++;
        ESP_LOGW(TAG, "local underrun, restarting timeline");

        s_underrun_recover = true;
        return CHUNK_UNDERRUN;
    }

    *got_frames = (uint32_t)(got / frame_bytes);
    if (got < want) {
        n_short_reads++;
        n_short_frames += (uint32_t)((want - got) / frame_bytes);
    }

    size_t have = got;
    if (cu && got == want) {

        audio_shift_chunk((int16_t *)chunk, s_cu_in, AUDIO_FRAMES,
                          shift, CATCHUP_FADE_FRAMES, AUDIO_CHANNELS);
        *got_frames = (uint32_t)((int)AUDIO_FRAMES + shift);
        have = sizeof(chunk);
        catchup_frames -= s_catchup_take;
        if (shift > 0) n_catchup_drops += (uint32_t)shift;
        else           n_catchup_dups  += (uint32_t)(-shift);

        s_catchup_moved += s_catchup_take < 0 ? -s_catchup_take
                                              : s_catchup_take;
        if (s_catchup_moved >= (int32_t)((int64_t)CATCHUP_HIST_RESET_US *
                                         sample_rate / 1000000)) {
            s_catchup_moved = 0;
            sync_phase_reset(&s_phase_hist);
            s_phase_med_valid = false;
            s_phase_stepped = true;
        }
    } else if (cu) {

        const size_t copy = got < sizeof(chunk) ? got : sizeof(chunk);
        memcpy(chunk, s_cu_in, copy);
        have = copy;
    }
    if (have < sizeof(chunk)) {
        memset(chunk + have, 0, sizeof(chunk) - have);
    }

    if (!cu && shift > 0) {

        static uint8_t dropped[AUDIO_CHANNELS * sizeof(int16_t)];
        if (xStreamBufferReceive(local_ring, dropped, sizeof(dropped), 0)
            == sizeof(dropped)) {
            *got_frames += 1;
            n_trim_drops++;
        } else {
            s_trim_owed += TRIM_ONE_FRAME;
        }
    } else if (!cu && shift < 0) {

        memcpy(chunk + want, chunk + want - frame_bytes, frame_bytes);
        n_trim_dups++;
    }
    return CHUNK_OK;
}

static void absorb_phase_crossings(void)
{

    while (s_phase_tail != s_phase_head && s_samples_played >= s_phase_q[s_phase_tail].pos) {

        int32_t overshoot = s_samples_played - s_phase_q[s_phase_tail].pos;

        if (overshoot > AUDIO_FRAMES) {
            overshoot = AUDIO_FRAMES;
        }

        const int64_t crossed_at = s_wrote_at
                                 - (int64_t)overshoot * 1000000 / sample_rate;
        const int32_t err = (int32_t)(crossed_at - s_phase_q[s_phase_tail].play_at);

        const int64_t since_retune = s_retune_done_at
                                   ? crossed_at - s_retune_done_at : -1;
        if (s_retune_watch) {
            s_retune_watch = false;
            ESP_LOGW(TAG, "RETUNE COST: phase %+ld -> %+ld us (net %+ld), "
                          "channel was down %lld us -- withheld from the "
                          "servo, crossed %lld us after the retune",
                     (long)s_retune_phase_before, (long)err,
                     (long)(err - s_retune_phase_before),
                     s_retune_outage_us, since_retune);
        } else {

            if (s_retune_tail_left) {
                s_retune_tail_left--;
                ESP_LOGW(TAG, "RETUNE TAIL: phase %+ld us at %lld us after "
                              "the retune (net %+ld from before it)",
                         (long)err, since_retune,
                         (long)(err - s_retune_phase_before));
            }

            if (s_refill_active) {
                n_refill_withheld++;
            } else {
                s_phase_err_us = err;
                s_phase_valid = true;
                sync_phase_push(&s_phase_hist, err);

                int32_t med;
                if (sync_phase_median(&s_phase_hist, &med)) {
                    s_phase_med_us = med;
                    s_phase_med_valid = true;
                }
            }
        }
        s_phase_tail = (s_phase_tail + 1) % PHASE_Q_LEN;
    }
}

static void apply_track_boundary(void)
{

    int32_t rp = s_restart_pos;
    if (rp >= 0 && s_samples_played >= rp) {
        s_restart_pos = -1;
        int32_t max_frames = (int32_t)sample_rate * MAX_SPLICE_MS / 1000;

        int32_t med_us = 0;
        const bool have_med = s_phase_valid &&
                              sync_phase_median(&s_phase_hist, &med_us);
        const int32_t splice_us = have_med ? med_us : (int32_t)s_phase_err_us;

        int32_t adj = s_phase_valid
            ? (int32_t)((int64_t)splice_us * sample_rate / 1000000) : 0;
        if (adj > max_frames)  adj = max_frames;
        if (adj < -max_frames) adj = -max_frames;
        int32_t applied = 0;

        int32_t raw_adj = s_phase_valid
            ? (int32_t)((int64_t)s_phase_err_us * sample_rate / 1000000) : 0;
        if (raw_adj > max_frames)  raw_adj = max_frames;
        if (raw_adj < -max_frames) raw_adj = -max_frames;
        s_hub_splice_alt_us = (int32_t)((int64_t)raw_adj * 1000000 / sample_rate);

        if (adj > 0) {

            static uint8_t discard[AUDIO_CHUNK_BYTES];
            int32_t left = adj;
            while (left > 0) {
                size_t want = (size_t)(left > AUDIO_FRAMES ? AUDIO_FRAMES : left)
                              * AUDIO_CHANNELS * sizeof(int16_t);
                size_t g = xStreamBufferReceive(local_ring, discard, want, 0);
                if (g == 0) break;
                left -= g / (AUDIO_CHANNELS * sizeof(int16_t));
            }
            s_samples_played += (adj - left);
            applied = adj - left;
            ESP_LOGW(TAG, "track boundary: skipped %ld ms to null phase",
                     (long)(applied * 1000 / (int32_t)sample_rate));
        } else if (adj < 0) {

            const int32_t level_frames = (int32_t)((LOCAL_RING_BYTES -
                    xStreamBufferSpacesAvailable(local_ring))
                    / (AUDIO_CHANNELS * (int)sizeof(int16_t)));
            const int32_t room = (int32_t)(LOCAL_RING_BYTES /
                    (AUDIO_CHANNELS * (int)sizeof(int16_t)))
                    - (int32_t)sample_rate * SPLICE_INSERT_HEADROOM_MS / 1000
                    - level_frames;
            if (-adj > room) {
                ESP_LOGW(TAG, "insert clamped: phase asked %ld ms, ring has "
                              "room for %ld ms",
                         (long)(-adj * 1000 / (int32_t)sample_rate),
                         (long)((room > 0 ? room : 0) * 1000
                                / (int32_t)sample_rate));
                adj = room > 0 ? -room : 0;
            }

            static const audio_out_sample_t quiet[AUDIO_FRAMES * AUDIO_CHANNELS] = {0};
            _Static_assert(sizeof(quiet) == AUDIO_OUT_CHUNK_BYTES,
                           "the splice's silence must be sized in output samples");
            int32_t left = -adj;
            size_t w = 0;
            while (left > 0) {
                int32_t n = left > AUDIO_FRAMES ? AUDIO_FRAMES : left;
                size_t bytes = (size_t)n * AUDIO_OUT_FRAME_BYTES;
                i2s_channel_write(i2s_tx, quiet, bytes, &w, portMAX_DELAY);
                left -= n;
            }
            applied = adj;
            ESP_LOGW(TAG, "track boundary: inserted %ld ms to null phase",
                     (long)(-applied * 1000 / (int32_t)sample_rate));
        }
        if (adj != 0) {
            n_splices++;
            s_phase_stepped = true;

            sync_phase_reset(&s_phase_hist);
            s_phase_med_valid = false;

            if (applied != 0) {
                catchup_frames = 0;
            }
        }

        s_hub_splice_us = (int32_t)((int64_t)applied * 1000000 / sample_rate);
        s_hub_splice_at = esp_timer_get_time();

        if (s_sync_at) {
            ESP_LOGW(TAG, "TRACK DIVERGENCE: satellite %+lld us "
                          "(marker, %lld ms before this boundary) | "
                          "hub spliced %+ld ms | hub phase %+ld us "
                          "(median %+ld us; raw would have spliced %+ld ms)",
                     s_sync_err_us,
                     (s_hub_splice_at - s_sync_at) / 1000,
                     (long)(s_hub_splice_us / 1000),
                     (long)s_phase_err_us,
                     (long)med_us, (long)(s_hub_splice_alt_us / 1000));
        } else {

            ESP_LOGW(TAG, "TRACK BOUNDARY: hub spliced %+ld ms | "
                          "hub phase %+ld us (median %+ld us; raw would "
                          "have spliced %+ld ms) | no marker fitted",
                     (long)(s_hub_splice_us / 1000),
                     (long)s_phase_err_us,
                     (long)med_us, (long)(s_hub_splice_alt_us / 1000));
        }

    }
}

static void pulse_marker(void)
{
    int32_t mark = s_marker_sample;
    if (mark >= 0 && s_samples_played >= mark) {
#if CONFIG_DANCEFLOOR_ENABLE_MARKER

        gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 1);
        s_marker_at = esp_timer_get_time();
        esp_rom_delay_us(MARKER_PULSE_US);
        gpio_set_level(CONFIG_DANCEFLOOR_MARKER_GPIO, 0);
#endif
        s_marker_sample = -1;
    }
}

static uint8_t vol_now(void)
{
    static bool told;
    const bool due = esp_timer_get_time() >= AUDIO_VOL_UNKNOWN_HOLD_US;
    if (due && !audio_vol_known && !told) {
        told = true;
        ESP_LOGE(TAG, "NO VOLUME in %d s -- nothing has told this unit a level. "
                      "Falling back to FULL SCALE.",
                 (int)(AUDIO_VOL_UNKNOWN_HOLD_US / 1000000));
    }
    return audio_vol_effective(audio_volume, audio_vol_known, due);
}

static audio_ramp_t s_out_ramp;

static void write_chunk(void)
{

    audio_apply_channel_mode((int16_t *)chunk, AUDIO_FRAMES);

    static audio_out_sample_t out32[AUDIO_FRAMES * AUDIO_CHANNELS];
    audio_volume_write_i32(out32, (const int16_t *)chunk, AUDIO_FRAMES, vol_now(),
                           &s_out_ramp);
    size_t written = 0;
    const int64_t w0 = s_refill_active ? esp_timer_get_time() : 0;
    if (i2s_channel_write(i2s_tx, out32, sizeof(out32), &written,
                          portMAX_DELAY) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (s_refill_active) {
        if (esp_timer_get_time() - w0 < REFILL_FAST_US) {
            s_refill_frames += (int32_t)(written / AUDIO_OUT_FRAME_BYTES);
        } else {
            s_refill_active = false;
            ESP_LOGW(TAG, "REFILL after start: %ld frames (%ld ms) before a "
                          "write blocked -- phase readings inside this "
                          "window are not DAC-paced",
                     (long)s_refill_frames,
                     (long)(s_refill_frames * 1000 / (int32_t)sample_rate));
        }
    }

    s_wrote_at = esp_timer_get_time();
}

static void begin_playback(void)
{

    s_out_ramp.cur = 0;

    s_samples_played = 0;

    s_trim_owed = 0;

    sync_phase_reset(&s_phase_hist);
    s_phase_med_valid = false;

    catchup_frames = 0;

    s_refill_active = true;
    s_refill_frames = 0;

    s_wrote_at = esp_timer_get_time();
}

void local_play_task(void *arg)
{
    (void)arg;

    while (1) {

        if (local_epoch == s_play_epoch) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        s_play_epoch = local_epoch;
        const int64_t start_at = local_start;

        int64_t wait = start_at - esp_timer_get_time();
        if (wait > 2000) {
            vTaskDelay(pdMS_TO_TICKS((wait - 2000) / 1000));
        }
        while (esp_timer_get_time() < start_at) {

        }

        ESP_LOGI(TAG, "local playback started: scheduled %lld, actual %lld (%+lld us)",
                 start_at, esp_timer_get_time(), esp_timer_get_time() - start_at);
        s_playing = true;
        begin_playback();

        while (1) {
            uint32_t got_frames = 0;

            if (park_for_retune()) {
                continue;
            }
            if (read_chunk(&got_frames) == CHUNK_UNDERRUN) {
                break;
            }
            absorb_phase_crossings();
            apply_track_boundary();
            pulse_marker();

            s_samples_played += (int32_t)got_frames;
            write_chunk();
        }

        s_playing = false;
    }
}
