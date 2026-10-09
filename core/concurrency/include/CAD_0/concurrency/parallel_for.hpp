// core/concurrency/include/CAD_0/concurrency/parallel_for.hpp
//
// Parallel iteration over a [0, n) range with a per-index callable.
// Uses oneTBB if available, otherwise a serial fallback.
//
#pragma once

#include <CAD_0/config.h>

#include <cstddef>
#include <iterator>
#include <type_traits>

#if defined(CAD_0_USE_TBB) && CAD_0_USE_TBB
#  include <tbb/parallel_for.h>
#  include <tbb/blocked_range.h>
#endif

namespace CAD_0::concurrency {

template<typename Index, typename Fn>
void parallel_for(Index begin, Index end, Fn&& fn) {
    if (end <= begin) return;
#if defined(CAD_0_USE_TBB) && CAD_0_USE_TBB
    // Pick a grain size so that small inputs (< 1024 elements) execute
    // serially to avoid thread-startup overhead.
    const Index n = end - begin;
    const Index grain = (n < 1024) ? n : 64;
    tbb::parallel_for(
        tbb::blocked_range<Index>(begin, end, grain),
        [&](const tbb::blocked_range<Index>& r) {
            for (Index i = r.begin(); i != r.end(); ++i) fn(i);
        });
#else
    for (Index i = begin; i < end; ++i) fn(i);
#endif
}

} // namespace CAD_0::concurrency
