/*
 * Where a slow analyser's answer waits for the moment it describes.
 *
 * The problem it solves: a model with a one-second context cannot answer until
 * a second of audio has arrived, which is long after the instant its window
 * began. Frames are drawn at the instant they name, so by then the frame that
 * window belongs to is already on the strip. Rather than delaying the strip --
 * which would delay the FFT's frames too, for an algorithm that does not need
 * it -- the result carries its own later instant, show_at_us, and waits here
 * until a frame with that due_us comes up for drawing.
 *
 * ---------------------------------------------------------------------------
 * Why this is latched at RENDER and not when the frame is produced
 * ---------------------------------------------------------------------------
 *
 * The analysis stage runs as far ahead as it has audio for -- up to the ~200 ms
 * of lead the hub puts on every packet -- so a frame due at T is produced at
 * about T-200 ms. Latching there would require every result to be ready 200 ms
 * before its own display time, spending the entire lead before the model had
 * started. The render stage reaches that frame at T. Latching there hands the
 * slow lane the whole lead as working time, which is the only reason a model
 * taking tens of milliseconds fits at all.
 *
 * ---------------------------------------------------------------------------
 * Why it stays in step across units
 * ---------------------------------------------------------------------------
 *
 * show_at_us is `window due_us + a compile-time constant` and a frame's due_us
 * is `index * HOP_N * 1e6 / rate`. Both are counted from an origin every unit
 * shares, so every unit latches the same result into the same frame index --
 * regardless of when its own inference actually finished, which is the one
 * thing about this that does differ per board.
 *
 * That holds only while a result ARRIVES before the frame it named is drawn.
 * One that does not lands in a later frame here and in the same later frame
 * nowhere else, so the strips differ until the next result. It is counted as
 * `late` rather than hidden, because the fix is a larger present_delay_us and
 * nothing else will say so.
 *
 * ---------------------------------------------------------------------------
 *
 * Here rather than inside the firmware because it is pure logic and it is the
 * most sync-critical thing in the analyser path -- the same reason analysis.cpp
 * is where it is. No clock, no task, no strip: `take()` is told the instant
 * rather than reading one. tools/pattern_lab and the unit tests drive exactly
 * this code.
 *
 * One producer per slot: an analyser lives in exactly one lane, and a unit is
 * either computing results or being sent them, never both. So each slot is a
 * plain single-producer/single-consumer ring and needs no lock.
 */
#pragma once

#include <stdint.h>

#include <atomic>

#include "analyser.hpp"

namespace df {

/*
 * How many results may wait in a slot.
 *
 * The producer runs ahead of the consumer by up to the presentation lead, so
 * this must cover every result an analyser can emit in that time. A slow
 * analyser reporting at 4 Hz emits one per 250 ms against a ~200 ms lead, so 16
 * is generous; it is sized for the analyser that reports fastest through a
 * latch, not the one that reports slowest.
 *
 * A fast-lane analyser never comes through here at all -- its result travels in
 * the frame -- which is what keeps this small.
 */
constexpr uint32_t LATCH_PENDING = 16;
static_assert((LATCH_PENDING & (LATCH_PENDING - 1)) == 0,
              "LATCH_PENDING must be a power of two -- the uint32 wrap depends on it");

class ResultLatch {
public:
    /*
     * Whether the consumer fills this slot from here, or finds it already in
     * the frame.
     *
     * Not simply "is the analyser slow". The question is whether the unit
     * produces that slot's result in the fast lane, and a unit taking results
     * off the wire produces none of them -- a fast analyser's results reach it
     * exactly like a slow one's, through here, because the network is the
     * delay. So this is false only for a slot the unit computes itself, inline,
     * per frame.
     *
     * Every slot starts latched, so a slot with no analyser behind it reports
     * result_none() forever rather than whatever the array happened to hold.
     */
    void set_latched(int slot, bool latched)
    {
        if (slot >= 0 && slot < ML_SLOTS) slot_[slot].latched = latched;
    }

    bool latched(int slot) const
    {
        return slot >= 0 && slot < ML_SLOTS ? slot_[slot].latched : true;
    }

    /* Producer side. Never blocks. False if the slot was full, which means the
     * consumer is not draining and is counted as an overrun. */
    bool publish(int slot, const Result &r)
    {
        if (slot < 0 || slot >= ML_SLOTS) return false;
        Slot &s = slot_[slot];
        const uint32_t head = s.head.load(std::memory_order_relaxed);
        const uint32_t tail = s.tail.load(std::memory_order_acquire);
        if (head - tail >= LATCH_PENDING) {
            overrun_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        s.ring[head % LATCH_PENDING] = r;
        /* Release, against the acquire in take(): the result must be fully
         * written before the index that publishes it moves. */
        s.head.store(head + 1, std::memory_order_release);
        return true;
    }

    /*
     * Consumer side: bring every latched slot up to `due_us` and report what it
     * holds.
     *
     * Consumes everything that has matured by this frame, keeping the last -- a
     * slot whose analyser reports faster than the strip draws should show its
     * newest answer, not work through a backlog of stale ones.
     *
     * `hop_us` is one frame period, and is only used to notice a result that
     * matured more than a frame before the frame now being drawn: that one
     * missed the frame it named, and every unit that got it on time drew it
     * earlier than this one will.
     *
     * Slots that are not latched are left untouched -- the caller already has
     * their results in the frame.
     */
    void take(int64_t due_us, int64_t hop_us, Result out[ML_SLOTS])
    {
        for (int i = 0; i < ML_SLOTS; i++) {
            Slot &s = slot_[i];
            if (!s.latched) continue;

            uint32_t tail = s.tail.load(std::memory_order_relaxed);
            const uint32_t head = s.head.load(std::memory_order_acquire);
            while (tail != head && s.ring[tail % LATCH_PENDING].show_at_us <= due_us) {
                s.current = s.ring[tail % LATCH_PENDING];
                if (hop_us > 0 && due_us - s.current.show_at_us >= hop_us) {
                    late_.fetch_add(1, std::memory_order_relaxed);
                }
                tail++;
            }
            s.tail.store(tail, std::memory_order_release);
            out[i] = s.current;
        }
    }

    /*
     * Drop everything waiting, keeping the latched/computed decision.
     *
     * Called on a flush, for the same reason the frame queue is emptied there:
     * a show_at_us established before a timeline restart names an instant on a
     * timeline that no longer exists, and latching it would put a stale answer
     * on the strip at the moment the new timeline starts.
     *
     * Consumer-side only -- it moves the tail, which is the consumer's index,
     * exactly as the frame queue's flush does.
     */
    void flush()
    {
        for (int i = 0; i < ML_SLOTS; i++) {
            slot_[i].tail.store(slot_[i].head.load(std::memory_order_acquire),
                                std::memory_order_release);
            slot_[i].current = result_none();
        }
    }

    /* Read-and-clear, for a periodic log line. */
    uint32_t take_late()    { return late_.exchange(0, std::memory_order_relaxed); }
    uint32_t take_overrun() { return overrun_.exchange(0, std::memory_order_relaxed); }

private:
    struct Slot {
        Result   ring[LATCH_PENDING];
        std::atomic<uint32_t> head{0};   /* the producing lane writes */
        std::atomic<uint32_t> tail{0};   /* the consumer writes */
        Result   current{result_none()}; /* consumer-owned */
        bool     latched{true};
    };

    Slot slot_[ML_SLOTS];
    std::atomic<uint32_t> late_{0};
    std::atomic<uint32_t> overrun_{0};
};

}  // namespace df
