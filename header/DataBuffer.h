#pragma once

#include <atomic>
#include <array>
#include <cstddef>

template<typename T, size_t Slots = 2>
class DataBuffer {
    static_assert(Slots >= 2, "Need at least double buffering");

public:
    DataBuffer() : write_idx_(0) {
    }

    // ── Producer side ───────────────────────────────────────────────
    // Returns pointer to the "next" inactive slot for direct writing
    // Thread-safe only if single producer per instance
    [[nodiscard("prepare_write return value cannot be ignored")]] T* prepare_write() noexcept {
        size_t next_idx = (write_idx_.load(std::memory_order_relaxed) + 1) % Slots;
        return &slots_[next_idx];
    }

    // After writing is complete, commit to buffer
    void commit() noexcept {
        size_t next_idx = (write_idx_.load(std::memory_order_relaxed) + 1) % Slots;
        write_idx_.store(next_idx, std::memory_order_release);
    }

    // ── Consumer / Reader side ──────────────────────────────────────
    // Lock-free, fast acquire semantic
    const T* get_latest() const noexcept {
        size_t idx = write_idx_.load(std::memory_order_acquire);
        return &slots_[idx];
    }

    // copy version if you need the data to survive longer
    T get_latest_copy() const {
        size_t idx = write_idx_.load(std::memory_order_acquire);
        return slots_[idx];
    }

private:
    std::array<T, Slots> slots_{};
    std::atomic<size_t>  write_idx_;
};
