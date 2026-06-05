cmake_minimum_required(VERSION 3.20)
project(YggApp LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/bigobj)
endif()

include(FetchContent)

# --- glad (local, pre-generated) ---
add_library(glad STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/src/glad.c
)
target_include_directories(glad PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glad/include
)

# --- GLFW ---
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

# --- Dear ImGui ---
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8
)
FetchContent_MakeAvailable(imgui)

add_library(imgui_impl STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_impl PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_impl PUBLIC glfw glad)

# --- OpenGL ---
find_package(OpenGL REQUIRED)

# --- OpenVDB (optional) ---
option(YGG_APP_USE_OPENVDB "Enable OpenVDB core integration" ON)

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
        ../core/SurfelGenerator.cpp
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
