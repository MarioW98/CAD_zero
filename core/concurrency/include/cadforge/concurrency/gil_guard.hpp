// core/concurrency/include/cadforge/concurrency/gil_guard.hpp
//
// RAII guard for the Python GIL when called from nanobind bindings.
// Allows C++ worker threads to release the GIL while they run, and
// reacquire it when they need to call back into Python.
//
// Usage:
//   {
//       cadforge::concurrency::gil_release no_gil;
//       // ... heavy C++ work ...
//   } // GIL reacquired here
//
// If the code is not linked against Python (no CADFORGE_BUILD_PYTHON),
// the guards are no-ops.
//
#pragma once

#include <cadforge/config.h>

#if defined(CADFORGE_BUILD_PYTHON) && CADFORGE_BUILD_PYTHON
#  include <Python.h>
#  define CADFORGE_HAS_PYTHON 1
#else
#  define CADFORGE_HAS_PYTHON 0
#endif

namespace cadforge::concurrency {

class gil_release {
public:
    gil_release() {
#if CADFORGE_HAS_PYTHON
        state_ = PyEval_SaveThread();
#endif
    }
    ~gil_release() {
#if CADFORGE_HAS_PYTHON
        if (state_) PyEval_RestoreThread(state_);
#endif
    }
    gil_release(const gil_release&) = delete;
    gil_release& operator=(const gil_release&) = delete;
private:
#if CADFORGE_HAS_PYTHON
    PyThreadState* state_{nullptr};
#endif
};

class gil_acquire {
public:
    gil_acquire() {
#if CADFORGE_HAS_PYTHON
        PyGILState_Ensure();
#endif
    }
    ~gil_acquire() {
#if CADFORGE_HAS_PYTHON
        PyGILState_Release(state_);
#endif
    }
    gil_acquire(const gil_acquire&) = delete;
    gil_acquire& operator=(const gil_acquire&) = delete;
private:
#if CADFORGE_HAS_PYTHON
    PyGILState_STATE state_{PyGILState_UNLOCKED};
#endif
};

} // namespace cadforge::concurrency
