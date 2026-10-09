// core/concurrency/include/CAD_0/concurrency/task_graph.hpp
//
// Task graph — used by the feature tree to express rebuild dependencies.
// Each feature declares its inputs (other features) and outputs (new
// shapes). The task graph executes the rebuild in topological order,
// parallelizing independent features.
//
// Implementation note: oneTBB::flow::graph is the right primitive but
// we use a thin wrapper so that the public API doesn't expose TBB types
// directly. This allows swapping the scheduler without touching callers.
//
#pragma once

#include <CAD_0/config.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace CAD_0::concurrency {

class TaskGraph {
public:
    TaskGraph();
    ~TaskGraph();

    TaskGraph(const TaskGraph&) = delete;
    TaskGraph& operator=(const TaskGraph&) = delete;

    // Add a node. Returns the node's index (used by add_edge).
    std::size_t add_node(std::function<void()> task);

    // Add a directed dependency: `producer` must complete before `consumer`.
    void add_edge(std::size_t producer, std::size_t consumer);

    // Execute the graph. Blocks until all tasks complete.
    void execute();

private:
    struct Impl;
    Impl* impl_;
};

} // namespace CAD_0::concurrency
