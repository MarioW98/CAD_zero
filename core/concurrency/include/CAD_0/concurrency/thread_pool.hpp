// core/concurrency/include/CAD_0/concurrency/thread_pool.hpp
//
// Thread pool abstraction. Default implementation uses oneTBB's
// task scheduler (see ADR-0009); when TBB is not available, falls back
// to a single-threaded shim so the build remains usable.
//
#pragma once

#include <CAD_0/config.h>

#include <cstddef>
#include <functional>

namespace CAD_0::concurrency {

class ThreadPool {
public:
    // Default constructor picks `hardware_concurrency()` threads.
    ThreadPool();
    explicit ThreadPool(std::size_t num_threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // Number of worker threads. Always >= 1.
    std::size_t size() const noexcept;

    // Submit a closure to be executed in parallel with `parallel_for`.
    // The closure will run on one of the pool's worker threads; this
    // function returns immediately. Use `parallel_for` for bulk work.
    void submit(std::function<void()> task);

private:
    struct Impl;
    Impl* impl_;
};

// Process-global default pool. Lazily initialized on first use.
ThreadPool& default_pool();

} // namespace CAD_0::concurrency
