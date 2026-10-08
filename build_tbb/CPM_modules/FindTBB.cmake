include("/home/z/my-project/CAD_0/build_tbb/cmake/CPM_0.40.2.cmake")
CPMAddPackage("NAME;TBB;GITHUB_REPOSITORY;oneapi-src/oneTBB;GIT_TAG;v2022.0.0;OPTIONS;TBB_TEST OFF;TBB_STRICT OFF")
set(TBB_FOUND TRUE)