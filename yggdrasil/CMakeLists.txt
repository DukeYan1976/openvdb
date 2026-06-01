cmake_minimum_required(VERSION 3.20)
project(Yggdrasil LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ensure GTest uses same standard
set(CMAKE_CXX_STANDARD 17 CACHE STRING "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Find OpenVDB (use project-local install)
set(OPENVDB_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../install")
set(CMAKE_MODULE_PATH "${OPENVDB_INSTALL_DIR}/lib/cmake/OpenVDB" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH "${OPENVDB_INSTALL_DIR}" ${CMAKE_PREFIX_PATH})
find_package(OpenVDB REQUIRED)

# Library
add_library(yggdrasil
    core/ResolutionSolver.cpp
    core/BilletBuilder.cpp
    core/ToolSweepSDF.cpp
    core/CuttingEngine.cpp
)
target_include_directories(yggdrasil PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(yggdrasil PUBLIC OpenVDB::openvdb)

# Tests
include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(ygg_tests
    tests/test_resolution.cpp
    tests/test_billet.cpp
    tests/test_toolsdf.cpp
    tests/test_cutting.cpp
)
target_link_libraries(ygg_tests PRIVATE yggdrasil GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(ygg_tests)
