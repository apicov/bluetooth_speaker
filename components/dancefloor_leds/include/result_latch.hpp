
#pragma once

#include <stdint.h>

#include <atomic>

#include "analyser.hpp"

namespace df {

constexpr uint32_t LATCH_PENDING = 16;
static_assert((LATCH_PENDING & (LATCH_PENDING - 1)) == 0,
              "LATCH_PENDING must be a power of two -- the uint32 wrap depends on it");

class ResultLatch {
public:

    void set_latched(int slot, bool latched)
    {
        if (slot >= 0 && slot < ML_SLOTS) slot_[slot].latched = latched;
    }

    bool latched(int slot) const
    {
        return slot >= 0 && slot < ML_SLOTS ? slot_[slot].latched : true;
    }

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

        s.head.store(head + 1, std::memory_order_release);
        return true;
    }

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

    void flush()
    {
        for (int i = 0; i < ML_SLOTS; i++) {
            slot_[i].tail.store(slot_[i].head.load(std::memory_order_acquire),
                                std::memory_order_release);
            slot_[i].current = result_none();
        }
    }

    uint32_t take_late()    { return late_.exchange(0, std::memory_order_relaxed); }
    uint32_t take_overrun() { return overrun_.exchange(0, std::memory_order_relaxed); }

private:
    struct Slot {
        Result   ring[LATCH_PENDING];
        std::atomic<uint32_t> head{0};
        std::atomic<uint32_t> tail{0};
        Result   current{result_none()};
        bool     latched{true};
    };

    Slot slot_[ML_SLOTS];
    std::atomic<uint32_t> late_{0};
    std::atomic<uint32_t> overrun_{0};
};

}
