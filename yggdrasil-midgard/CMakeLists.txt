cmake_minimum_required(VERSION 3.16)
project(yggdrasil-midgard LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ═══════════════════════════════════════════════════════════════
# 选项
# ═══════════════════════════════════════════════════════════════
option(MIDGARD_DEV   "Enable debug UI"          ON)
option(MIDGARD_APP   "Build IMGUI application"   ON)
option(MIDGARD_TESTS "Build GTest-based tests"   ON)

# ═══════════════════════════════════════════════════════════════
# 依赖查找 (跨平台: CMake targets → 手动路径 fallback)
# ═══════════════════════════════════════════════════════════════

# -- OpenVDB --
find_package(OpenVDB QUIET CONFIG)
if(OpenVDB_FOUND AND TARGET OpenVDB::openvdb)
    set(OVDB_LIBS OpenVDB::openvdb)
    get_target_property(OVDB_INC OpenVDB::openvdb INTERFACE_INCLUDE_DIRECTORIES)
else()
    find_path(OVDB_INC openvdb/openvdb.h
        PATHS /opt/homebrew/opt/openvdb/include /usr/local/include /usr/include)
    find_library(OVDB_LIB openvdb
        PATHS /opt/homebrew/opt/openvdb/lib /usr/local/lib /usr/lib)
    if(NOT OVDB_INC OR NOT OVDB_LIB)
        message(FATAL_ERROR "OpenVDB not found")
    endif()
    set(OVDB_LIBS ${OVDB_LIB})
endif()

# -- TBB --
find_package(TBB QUIET CONFIG)
if(TBB_FOUND AND TARGET TBB::tbb)
    set(TBB_LIBS TBB::tbb)
    get_target_property(TBB_INC TBB::tbb INTERFACE_INCLUDE_DIRECTORIES)
else()
    find_path(TBB_INC tbb/tbb.h
        PATHS /opt/homebrew/opt/tbb/include /usr/local/include /usr/include)
    find_library(TBB_LIB tbb
        PATHS /opt/homebrew/opt/tbb/lib /usr/local/lib /usr/lib)
    if(NOT TBB_INC OR NOT TBB_LIB)
        message(FATAL_ERROR "TBB not found")
    endif()
    set(TBB_LIBS ${TBB_LIB})
endif()

# -- GTest --
if(MIDGARD_TESTS)
    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        message(WARNING "GTest not found — tests disabled")
        set(MIDGARD_TESTS OFF)
    endif()
endif()

# ═══════════════════════════════════════════════════════════════
# midgard-core (静态库)
# ═══════════════════════════════════════════════════════════════
add_library(midgard-core STATIC
    core/IPWBuilder.cpp
    core/IDebugDisplay.cpp
    core/MacroCut.cpp
    core/MicroCut.cpp
    core/MicroGridLab.cpp
    core/OctreeRefiner.cpp
    core/ToolSweptSDF.cpp
    core/ToolSweepSurface.cpp
    core/LocalSurfaceEngine.cpp
    core/SimEngine.cpp
    core/SurfaceMesher.cpp
    debug/RtDebugSys.cpp
)
target_compile_definitions(midgard-core PRIVATE MIDGARD_DEV)
if(MSVC)
    target_compile_definitions(midgard-core PUBLIC _USE_MATH_DEFINES)
    target_compile_options(midgard-core PRIVATE /bigobj)
endif()
target_include_directories(midgard-core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(midgard-core PUBLIC ${OVDB_INC} ${TBB_INC})
# (link dirs handled by targets/find_library; empty on config)
target_link_libraries(midgard-core PUBLIC ${OVDB_LIBS} ${TBB_LIBS})

# ═══════════════════════════════════════════════════════════════
# midgard-app (IMGUI + GLFW + OpenGL)
# ═══════════════════════════════════════════════════════════════
if(MIDGARD_APP)
    # GLFW — system or yggdrasil-local
    find_package(glfw3 QUIET)
    if(glfw3_FOUND AND TARGET glfw)
        set(GLFW_LIBS glfw)
    else()
        find_library(GLFW_LIB glfw3 glfw
            PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../yggdrasil/app/build/_deps/glfw-build/src
                  /opt/homebrew/opt/glfw/lib
                  /usr/local/lib /usr/lib)
        if(GLFW_LIB)
            set(GLFW_LIBS ${GLFW_LIB})
        endif()
    endif()

    # IMGUI — 可配置路径，默认 yggdrasil 本地构建
    set(IMGUI_INC "${CMAKE_CURRENT_SOURCE_DIR}/../yggdrasil/app/build/_deps/imgui-src"
        CACHE PATH "IMGUI include directory")
    set(IMGUI_BE  "${CMAKE_CURRENT_SOURCE_DIR}/../yggdrasil/app/build/_deps/imgui-src/backends"
        CACHE PATH "IMGUI backends directory")
    if(WIN32)
        set(_IMGUI_LIB_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/../yggdrasil/app/build/imgui_impl.lib")
    else()
        set(_IMGUI_LIB_DEFAULT "${CMAKE_CURRENT_SOURCE_DIR}/../yggdrasil/app/build/libimgui_impl.a")
    endif()
    set(IMGUI_LIB "${_IMGUI_LIB_DEFAULT}" CACHE FILEPATH "IMGUI impl library")

    # glad — project-local
    set(GLAD_INC "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include")
    set(GLAD_SRC "${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/src/glad.c")

    if(GLFW_LIBS AND EXISTS ${IMGUI_INC}/imgui.h AND EXISTS ${GLAD_SRC})
        add_executable(midgard-app
            app/main.cpp
            app/AppState.cpp
            app/windows/SettingsWindow.cpp
            app/windows/SimControlWindow.cpp
            app/windows/OutputWindow.cpp
            app/windows/DebugWindow.cpp
            app/windows/MicroGridLabWindow.cpp
            app/renderers/GPURenderers.cpp
            app/renderers/Camera.cpp
            app/renderers/ShaderProgram.cpp
            app/renderers/BilletMeshGenerator.cpp
            app/renderers/ToolMeshGenerator.cpp
            app/renderers/SceneRenderer.cpp
            ${GLAD_SRC}
        )
        target_compile_definitions(midgard-app PRIVATE MIDGARD_DEV)
        if(MSVC)
            target_compile_options(midgard-app PRIVATE /bigobj)
        endif()
        target_include_directories(midgard-app PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/app
            ${CMAKE_CURRENT_SOURCE_DIR}/app/windows
            ${CMAKE_CURRENT_SOURCE_DIR}/app/renderers
            ${GLAD_INC} ${IMGUI_INC} ${IMGUI_BE}
        )
        target_link_libraries(midgard-app PRIVATE
            midgard-core ${GLFW_LIBS} ${IMGUI_LIB}
        )
        if(APPLE)
            target_link_libraries(midgard-app PRIVATE
                "-framework OpenGL" "-framework Cocoa"
                "-framework IOKit" "-framework CoreFoundation")
        elseif(UNIX)
            target_link_libraries(midgard-app PRIVATE ${CMAKE_DL_LIBS} pthread)
        elseif(WIN32)
            target_link_libraries(midgard-app PRIVATE opengl32)
        endif()
    else()
        message(WARNING "midgard-app disabled: GLFW/IMGUI/glad not found")
    endif()
endif()

# ═══════════════════════════════════════════════════════════════
# 测试
# ═══════════════════════════════════════════════════════════════
if(MIDGARD_TESTS)
    enable_testing()
    add_executable(midgard_tests
        tests/test_sim_engine.cpp
        tests/test_microgrid_lab.cpp
        tests/test_toolsweep_surface.cpp
        tests/test_surfacemesher.cpp
    )
    target_link_libraries(midgard_tests PRIVATE
        midgard-core GTest::gtest GTest::gtest_main
    )
    add_test(NAME midgard_tests_run COMMAND midgard_tests)
endif()

# ═══════════════════════════════════════════════════════════════
# 摘要
# ═══════════════════════════════════════════════════════════════
message(STATUS "yggdrasil-midgard: App=${MIDGARD_APP} Tests=${MIDGARD_TESTS}")
