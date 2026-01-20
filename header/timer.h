#pragma once

#include <chrono>
#include <pthread.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <functional>
#include <memory>

class Timer {
public:
    enum class TimeUnit { Seconds, Milliseconds, Microseconds, Nanoseconds };
    using Callback = std::move_only_function<void()>;

    // Generic constructor
    template <typename Rep, typename Period>
    explicit Timer(std::chrono::duration<Rep, Period> interval, bool one_shot = false)
        : interval_(std::chrono::duration_cast<std::chrono::microseconds>(interval)),
          one_shot_(one_shot),
          running_(false),
          should_fire_(false),
          last_time_(std::chrono::steady_clock::now())
    {
        if (interval_ <= std::chrono::microseconds(0)) {
            std::cout << "Interval must be positive\n";
        }
    }

    ~Timer() {
        stop();
    }

    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;

    // Set or replace callback (thread-safe, non-blocking)
    void set_callback(Callback cb) {
        callback_.store(std::make_shared<Callback>(std::move(cb)),std::memory_order_release);
    }

    // Clear callback
    void clear_callback() {
        callback_.store(nullptr, std::memory_order_release);
    }

    void start(bool use_real_time_priority = false) {
        if (running_) {
            std::cout << "Timer already running\n";
            return;
        }

        running_ = true;
        started_ = true;
        should_fire_ = false;
        last_time_ = std::chrono::steady_clock::now();

        timer_thread_ = std::thread([this, use_real_time_priority]() {
        #ifdef __linux__
            if (use_real_time_priority) {
                sched_param param{};
                param.sched_priority = 20;
                if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
                    std::cout << "Warning: Failed to set real-time priority\n";
                }
            }
        #endif
            auto next_time = std::chrono::steady_clock::now() + interval_;

            while (running_) {
                std::this_thread::sleep_until(next_time);
                if (!running_) break;

                should_fire_ = true;

                // Invoke callback if present
                auto cb = callback_.load(std::memory_order_acquire);
                if (cb) {
                    try {
                        (*cb)();  // must be non-blocking
                    } catch (...) {
                        std::cout << "[Timer] callback threw an exception\n";
                    }
                }

                if (one_shot_) {
                    running_ = false;
                    break;
                }

                // Drift correction
                auto now = std::chrono::steady_clock::now();
                while (next_time + interval_ <= now) {
                    next_time += interval_;
                }

                next_time += interval_;
            }
        });
    }

    void stop() {
        if (!running_ && !started_) return;

        running_ = false;
        should_fire_ = false;

        if (started_ &&
            timer_thread_.joinable() &&
            std::this_thread::get_id() != timer_thread_.get_id()) {
            timer_thread_.join();
        }

        started_ = false;
    }

    bool check() {
        return should_fire_.exchange(false);
    }

    void set(bool val) {
        should_fire_.store(val, std::memory_order_release);
    }
    double get_elapsed_time(TimeUnit unit = TimeUnit::Microseconds) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = now - last_time_;
        last_time_ = now;

        switch (unit) {
            case TimeUnit::Seconds:
                return std::chrono::duration<double>(elapsed).count();
            case TimeUnit::Milliseconds:
                return std::chrono::duration<double, std::milli>(elapsed).count();
            case TimeUnit::Microseconds:
                return std::chrono::duration<double, std::micro>(elapsed).count();
            case TimeUnit::Nanoseconds:
                return std::chrono::duration<double, std::nano>(elapsed).count();
            default:
                return 0.0;
        }
    }

    bool is_running() const {
        return running_;
    }

    template <typename Rep, typename Period>
    void set_interval(std::chrono::duration<Rep, Period> new_interval) {
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(new_interval);
        if (us <= std::chrono::microseconds(0)) {
            std::cout << "Interval must be positive\n";
        }
        interval_ = us;
    }

private:
    std::chrono::microseconds interval_;
    bool one_shot_;
    bool started_ = false;
    std::atomic<bool> running_;
    std::atomic<bool> should_fire_;
    std::atomic<std::shared_ptr<Callback>> callback_{nullptr};
    std::thread timer_thread_;
    std::chrono::steady_clock::time_point last_time_;
};
