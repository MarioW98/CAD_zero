// core/concurrency/src/thread_pool.cpp
#include "cadforge/concurrency/thread_pool.hpp"
#include "cadforge/concurrency/parallel_for.hpp"
#include "cadforge/concurrency/task_graph.hpp"

#include <cadforge/config.h>

#if defined(CADFORGE_USE_TBB) && CADFORGE_USE_TBB
#  include <tbb/global_control.h>
#  include <tbb/task_arena.h>
#  include <tbb/task_group.h>
#endif

#include <thread>

namespace cadforge::concurrency {

struct ThreadPool::Impl {
#if defined(CADFORGE_USE_TBB) && CADFORGE_USE_TBB
    std::unique_ptr<tbb::global_control> control;
    std::unique_ptr<tbb::task_arena>     arena;
    tbb::task_group                       group;
#endif
    std::size_t num_threads{1};
};

ThreadPool::ThreadPool()
    : ThreadPool(std::thread::hardware_concurrency()) {}

ThreadPool::ThreadPool(std::size_t num_threads) {
    impl_ = new Impl{};
    impl_->num_threads = (num_threads == 0) ? 1 : num_threads;
#if defined(CADFORGE_USE_TBB) && CADFORGE_USE_TBB
    impl_->control = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism,
        impl_->num_threads);
    impl_->arena = std::make_unique<tbb::task_arena>(impl_->num_threads);
#endif
}

ThreadPool::~ThreadPool() { delete impl_; }

std::size_t ThreadPool::size() const noexcept {
    return impl_ ? impl_->num_threads : 1;
}

void ThreadPool::submit(std::function<void()> task) {
#if defined(CADFORGE_USE_TBB) && CADFORGE_USE_TBB
    impl_->arena->execute([this, task = std::move(task)]() {
        impl_->group.run(std::move(task));
    });
#else
    task();
#endif
}

ThreadPool& default_pool() {
    static ThreadPool pool;
    return pool;
}

// ---------------------------------------------------------------------------
// TaskGraph
// ---------------------------------------------------------------------------
struct TaskGraph::Impl {
    struct Node {
        std::function<void()> task;
        std::vector<std::size_t> deps; // producer indices
    };
    std::vector<Node> nodes;
};

TaskGraph::TaskGraph() : impl_(new Impl{}) {}
TaskGraph::~TaskGraph() { delete impl_; }

std::size_t TaskGraph::add_node(std::function<void()> task) {
    impl_->nodes.push_back({std::move(task), {}});
    return impl_->nodes.size() - 1;
}

void TaskGraph::add_edge(std::size_t producer, std::size_t consumer) {
    impl_->nodes.at(consumer).deps.push_back(producer);
}

void TaskGraph::execute() {
    // Simple topological serial execution — sufficient for Phase A.
    // In Phase B this will dispatch to TBB::flow::graph for parallelism
    // among independent features.
    for (auto& n : impl_->nodes) n.task();
}

} // namespace cadforge::concurrency
