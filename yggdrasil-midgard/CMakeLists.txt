cmake_minimum_required(VERSION 3.20)
project(YggdrasilMidgard LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/bigobj)
endif()

# OpenVDB (project-local install)
set(OPENVDB_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../install")
set(CMAKE_MODULE_PATH "${OPENVDB_INSTALL_DIR}/lib/cmake/OpenVDB" ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH "${OPENVDB_INSTALL_DIR}" ${CMAKE_PREFIX_PATH})
find_package(OpenVDB REQUIRED)

# Library
add_library(midgard
    debug/RtDebugSys.cpp
    core/ToolSweepSDF.cpp
    core/MacroCut.cpp
    core/IPWBuilder.cpp
)
target_include_directories(midgard PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(midgard PUBLIC OpenVDB::openvdb)

# App
add_executable(midgard_app app/main.cpp)
target_link_libraries(midgard_app PRIVATE midgard)

# Tests
include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(midgard_tests
    tests/test_placeholder.cpp
    tests/test_toolsweep_sdf.cpp
    tests/test_macrocut.cpp
    tests/test_ipwbuilder.cpp
)
target_link_libraries(midgard_tests PRIVATE midgard GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(midgard_tests)
