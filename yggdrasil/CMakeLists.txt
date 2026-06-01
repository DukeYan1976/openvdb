cmake_minimum_required(VERSION 3.20)
project(Yggdrasil LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library (R1 不依赖 OpenVDB 运行时)
add_library(yggdrasil
    core/ResolutionSolver.cpp
)
target_include_directories(yggdrasil PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# OpenVDB (可选，后续 Sprint 启用)
find_package(OpenVDB QUIET)
if(OpenVDB_FOUND)
    target_link_libraries(yggdrasil PUBLIC OpenVDB::openvdb)
    target_compile_definitions(yggdrasil PUBLIC YGG_HAS_OPENVDB=1)
endif()

# Tests
include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(ygg_tests
    tests/test_resolution.cpp
)
target_link_libraries(ygg_tests PRIVATE yggdrasil GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(ygg_tests)
