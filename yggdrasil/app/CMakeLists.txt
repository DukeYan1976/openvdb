cmake_minimum_required(VERSION 3.20)
project(YggApp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# --- glad (local, pre-generated) ---
add_library(glad STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/src/glad.c
)
target_include_directories(glad PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include
)

# --- GLFW (use existing local source) ---
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
    SOURCE_DIR "D:/WorkSpace/02_Develop/SourceCode_2/2DRegionApprox/build/_deps/glfw-src"
)
FetchContent_MakeAvailable(glfw)

# --- Dear ImGui (use existing local source) ---
set(IMGUI_DIR "D:/WorkSpace/02_Develop/SourceCode_2/2DRegionApprox/build/_deps/imgui-src")

add_library(imgui_impl STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_impl PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
target_link_libraries(imgui_impl PUBLIC glfw glad)

# --- OpenGL ---
find_package(OpenGL REQUIRED)

# --- OpenVDB (optional) ---
option(YGG_APP_USE_OPENVDB "Enable OpenVDB core integration" OFF)

set(APP_SOURCES main.cpp)
set(APP_LIBS imgui_impl glad OpenGL::GL)
set(APP_DEFINES "")

if(YGG_APP_USE_OPENVDB)
    set(OPENVDB_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install"
        CACHE PATH "OpenVDB install directory")
    list(APPEND CMAKE_MODULE_PATH "${OPENVDB_INSTALL_DIR}/lib/cmake/OpenVDB")
    list(APPEND CMAKE_PREFIX_PATH "${OPENVDB_INSTALL_DIR}")
    find_package(OpenVDB REQUIRED)

    add_library(ygg_core STATIC
        ../core/ResolutionSolver.cpp
        ../core/BilletBuilder.cpp
        ../core/ToolSweepSDF.cpp
        ../core/CuttingEngine.cpp
    )
    target_include_directories(ygg_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
    target_link_libraries(ygg_core PUBLIC OpenVDB::openvdb)

    list(APPEND APP_SOURCES VDBToMesh.cpp)
    list(APPEND APP_LIBS ygg_core)
    list(APPEND APP_DEFINES YGG_HAS_OPENVDB=1)
endif()

# --- App executable ---
add_executable(ygg_app ${APP_SOURCES})
target_link_libraries(ygg_app PRIVATE ${APP_LIBS})
target_compile_definitions(ygg_app PRIVATE ${APP_DEFINES})
