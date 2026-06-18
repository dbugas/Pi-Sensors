#pragma once

#include <atomic>
#include <array>
#include <cstddef>

// Single-producer, multiple-consumer lock-free ring buffer.
// Only one thread may call prepare_write/commit at a time.
// Multiple threads may call get_latest concurrently.
template<typename T, size_t Slots = 3>
class DataBuffer {
    static_assert(Slots >= 2, "Need at least 2 slots");
public:
    DataBuffer() : write_idx_(0) {}

    // Producer side — call prepare_write, fill the slot, then commit. Never
    // call prepare_write a second time before committing the first.
    [[nodiscard]] T* prepare_write() noexcept {
        pending_idx_ = (write_idx_.load(std::memory_order_relaxed) + 1) % Slots;
        return &slots_[pending_idx_];
    }

    void commit() noexcept {
        // Release so the reader's acquire sees the completed write.
        write_idx_.store(pending_idx_, std::memory_order_release);
    }

    // Consumer side — lock-free, safe to call from any thread at any time.
    const T* get_latest() const noexcept {
        return &slots_[write_idx_.load(std::memory_order_acquire)];
    }

    T get_latest_copy() const noexcept {
        return slots_[write_idx_.load(std::memory_order_acquire)];
    }

private:
    std::array<T, Slots> slots_{};
    std::atomic<size_t>  write_idx_;
    size_t               pending_idx_{1}; // only touched by the producer thread
};
