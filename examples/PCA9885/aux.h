#pragma once

#include <vector>
#include <queue>             
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <memory>
#include <type_traits>
#include <atomic>
#include <algorithm>         
#include <chrono>
#include <iostream>
#include <linux/input.h>
#include <unordered_map>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <poll.h>

#include "pigpio.h"

class ThreadManager {
public:
    // Construct with number of worker threads
    explicit ThreadManager(size_t numThreads = std::thread::hardware_concurrency())
        : stop(false) {
        workers.reserve(numThreads);
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this](std::stop_token stoken) {worker(stoken); });
        }
    }

    ~ThreadManager() {
        stop.store(true, std::memory_order_relaxed);
        cv.notify_all();
        // jthreads join automatically on destruction
    }

    // Submit a task with priority
    // Returns a future to the result
    template <typename Function, typename... Args>
    auto addTask(int priority, Function&& func, Args&&... args)
        -> std::future<std::invoke_result_t<std::decay_t<Function>, std::decay_t<Args>...>>
    {
        using ReturnType = std::invoke_result_t<std::decay_t<Function>, std::decay_t<Args>...>;

        // Create packaged task to capture result and exceptions
        auto task = std::make_shared<std::packaged_task<ReturnType()>>(
            [f = std::forward<Function>(func), ...args = std::forward<Args>(args)]() mutable {
                return f(std::move(args)...);
            });

        std::future<ReturnType> result = task->get_future();

        {
            std::lock_guard<std::mutex> lock(mutex);
            taskHeap.emplace_back(std::make_unique<Task>(
                priority,
                [task = std::move(task)]() mutable { (*task)(); }
            ));
            std::push_heap(taskHeap.begin(), taskHeap.end(), TaskCompare{});
        }

        cv.notify_one();
        return result;
    }

    // Wait until all currently queued tasks are completed
    // (Does NOT wait for new tasks submitted after this call)
    void waitAll() {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this] {
            return taskHeap.empty() && !stop.load(std::memory_order_relaxed);
            });
    }

private:
    struct Task {
        int priority;
        std::move_only_function<void()> func;

        Task(int p, std::move_only_function<void()> f)
            : priority(p), func(std::move(f)) {
        }
    };

    struct TaskCompare {
        bool operator()(const std::unique_ptr<Task>& a,
            const std::unique_ptr<Task>& b) const noexcept {
            return a->priority < b->priority; // max-heap: higher number = higher priority
        }
    };

    std::vector<std::jthread> workers;
    std::vector<std::unique_ptr<Task>> taskHeap;
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<bool> stop{ false };

    void worker(std::stop_token stoken) {
        while (!stoken.stop_requested()) {
            std::unique_ptr<Task> task;

            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [this, stoken] {
                    return stop.load(std::memory_order_relaxed) ||
                        !taskHeap.empty() ||
                        stoken.stop_requested();
                    });

                if (stoken.stop_requested() || (stop.load(std::memory_order_relaxed) && taskHeap.empty())) {
                    return;
                }

                // Extract highest priority task
                std::pop_heap(taskHeap.begin(), taskHeap.end(), TaskCompare{});
                task = std::move(taskHeap.back());
                taskHeap.pop_back();
            }

            // Execute the task (exceptions are captured by packaged_task)
            task->func();
        }
    }
};

class KeyboardIO {
private:
    int fd;
    std::string devicePath;
    struct pollfd fds;
    int ret;
    const int timeout_ms;
    std::mutex queueMutex;
    std::queue<int> eventQueue;  // Queue to store key events
    bool running;

    // Key map for human-readable key names
    std::unordered_map<int, std::string> keyMap = {
        {KEY_A, "A"}, {KEY_B, "B"}, {KEY_C, "C"}, {KEY_D, "D"},
        {KEY_E, "E"}, {KEY_F, "F"}, {KEY_G, "G"}, {KEY_H, "H"},
        {KEY_I, "I"}, {KEY_J, "J"}, {KEY_K, "K"}, {KEY_L, "L"},
        {KEY_M, "M"}, {KEY_N, "N"}, {KEY_O, "O"}, {KEY_P, "P"},
        {KEY_Q, "Q"}, {KEY_R, "R"}, {KEY_S, "S"}, {KEY_T, "T"},
        {KEY_U, "U"}, {KEY_V, "V"}, {KEY_W, "W"}, {KEY_X, "X"},
        {KEY_Y, "Y"}, {KEY_Z, "Z"},
        {KEY_1, "1"}, {KEY_2, "2"}, {KEY_3, "3"}, {KEY_4, "4"},
        {KEY_5, "5"}, {KEY_6, "6"}, {KEY_7, "7"}, {KEY_8, "8"},
        {KEY_9, "9"}, {KEY_0, "0"},
        {KEY_SPACE, "SPACE"}, {KEY_ENTER, "ENTER"}, {KEY_ESC, "ESC"},
        {KEY_TAB, "TAB"}, {KEY_BACKSPACE, "BACKSPACE"},
        {KEY_LEFT, "LEFT ARROW"}, {KEY_RIGHT, "RIGHT ARROW"},
        {KEY_UP, "UP ARROW"}, {KEY_DOWN, "DOWN ARROW"},
        {KEY_F1, "F1"}, {KEY_F2, "F2"}, {KEY_F3, "F3"}, {KEY_F4, "F4"},
        {KEY_F5, "F5"}, {KEY_F6, "F6"}, {KEY_F7, "F7"}, {KEY_F8, "F8"},
        {KEY_F9, "F9"}, {KEY_F10, "F10"}, {KEY_F11, "F11"}, {KEY_F12, "F12"}
    };

    input_event readKeyboard() {
        input_event inputData{};
        ssize_t bytesRead = read(fd, &inputData, sizeof(inputData));

        if (bytesRead < 0) {
            std::cerr << "Read error: " << strerror(errno) << std::endl;
        } else if (bytesRead != sizeof(inputData)) {
            std::cerr << "Incomplete event read" << std::endl;
        }

        return inputData;
    }

    bool pollEvent() {
        ret = poll(&fds, 1, timeout_ms);
        return (ret > 0 && (fds.revents & POLLIN));
    }

    void pushKeyEvent(int keyCode) {
        std::lock_guard<std::mutex> lock(queueMutex);
        eventQueue.push(keyCode);
    }
public:
    explicit KeyboardIO(const std::string& inputDevice, int timeout = 5000)
        : devicePath(inputDevice), ret(0), timeout_ms(timeout), running(true) {
        fd = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "Error opening device " << devicePath << ": " << strerror(errno) << std::endl;
        } else {
            std::cout << "Listening for keyboard events on " << devicePath << "..." << std::endl;
            fds.fd = fd;
            fds.events = POLLIN;
        }
    }

    void listenForEvents() {
        while(running) {
            if (pollEvent()) {
                input_event event = readKeyboard();
                if (event.type == EV_KEY && event.value == 1) {  // Key press event
                    pushKeyEvent(event.code);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::string getKeyName(int keyCode) {
        if (keyMap.find(keyCode) != keyMap.end()) {
            return keyMap[keyCode];
        }
        return "Unknown Key (" + std::to_string(keyCode) + ")";
    }


    int getKeyEvent() {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (!eventQueue.empty()) {
            int key = eventQueue.front();
            eventQueue.pop();
            return key;
        }
        return -1;  // No event
    }

    bool isOpen() const {
        return fd >= 0;
    }
    void stopListening() {
        running = false;
        // The ThreadManager destructor will ensure all threads are joined
    }
    ~KeyboardIO() {
        if (fd >= 0) {
            close(fd);
        }
    }
};