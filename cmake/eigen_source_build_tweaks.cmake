set(CMAKE_REQUIRED_QUIET ON "Silence checks in this function.")

set(EIGEN_BUILD_DOC
    OFF
    CACHE BOOL "Disable building docs for eigen" FORCE)
set(EIGEN_TEST_NOQT
    ON
    CACHE BOOL "Disable building qt tests for eigen" FORCE)
set(EIGEN_BUILD_TESTING
    OFF
    CACHE BOOL "Disable building tests for eigen" FORCE)
