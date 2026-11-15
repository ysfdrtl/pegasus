cmake_minimum_required(VERSION 3.20)
project(Pegasus VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Vulkan REQUIRED)

include(FetchContent)
set(FETCHCONTENT_QUIET ON)

# --- GLFW (system package if available, otherwise fetched) -------------------
find_package(glfw3 3.3 QUIET)
if(NOT glfw3_FOUND)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4 GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(glfw)
endif()

# --- GLM ---------------------------------------------------------------------
FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.1 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(glm)

# --- Dear ImGui (docking) + ImGuizmo: sources only, we build them ourselves ---
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.92.9b-docking
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR _no_cmake_)
FetchContent_Declare(imguizmo
  GIT_REPOSITORY https://github.com/CedricGuillemet/ImGuizmo.git
  GIT_TAG 105e4fde45e4e74aae8973d98750820ef7aca7e2
  SOURCE_SUBDIR _no_cmake_)
FetchContent_MakeAvailable(imgui imguizmo)

add_library(imgui_lib STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
  ${imguizmo_SOURCE_DIR}/src/ImGuizmo.cpp)
target_include_directories(imgui_lib PUBLIC
  ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${imguizmo_SOURCE_DIR}/src)
target_link_libraries(imgui_lib PUBLIC Vulkan::Vulkan glfw)

# --- Shaders -----------------------------------------------------------------
find_program(GLSLC glslc HINTS $ENV{VULKAN_SDK}/bin REQUIRED)
set(SHADER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shaders)
set(SHADER_OUT_DIR ${CMAKE_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${SHADER_OUT_DIR})
set(SHADERS mesh.vert mesh.frag line.vert line.frag)
set(SPIRV_FILES)
foreach(SH ${SHADERS})
  set(OUT ${SHADER_OUT_DIR}/${SH}.spv)
  add_custom_command(OUTPUT ${OUT}
    COMMAND ${GLSLC} -O ${SHADER_SRC_DIR}/${SH} -o ${OUT}
    DEPENDS ${SHADER_SRC_DIR}/${SH}
    COMMENT "Compiling shader ${SH}")
  list(APPEND SPIRV_FILES ${OUT})
endforeach()
add_custom_target(pegasus_shaders DEPENDS ${SPIRV_FILES})

# --- Application ---------------------------------------------------------------
add_executable(pegasus
  src/main.cpp
  src/App.cpp
  src/render/VulkanContext.cpp
  src/render/Renderer.cpp
  src/render/Png.cpp
  src/scene/Mesh.cpp
  src/scene/StlLoader.cpp
  src/scene/Primitives.cpp
  src/scene/Scene.cpp
  src/geom/Intersect.cpp
  src/geom/Collision.cpp
  src/robot/Robot.cpp
  src/robot/JointDetect.cpp
  src/robot/IkSolver.cpp
  src/motion/Expr.cpp
  src/motion/MotionLibrary.cpp
  src/motion/Seam.cpp
  src/motion/PathPlanner.cpp
  src/motion/SurfaceScan.cpp
  src/editor/Camera.cpp
  src/editor/Picking.cpp
  src/editor/Editor.cpp
  src/editor/Editor_Robot.cpp
  src/editor/Editor_Motion.cpp
  src/editor/Editor_Select.cpp
  src/editor/Editor_Arduino.cpp
  src/hw/SerialPort.cpp
  src/hw/ArduinoLink.cpp
  src/editor/FileBrowser.cpp
  src/editor/SelfTest.cpp)
add_dependencies(pegasus pegasus_shaders)
target_include_directories(pegasus PRIVATE src)
find_package(Threads REQUIRED)
target_link_libraries(pegasus PRIVATE imgui_lib glm::glm Vulkan::Vulkan glfw Threads::Threads)
target_compile_definitions(pegasus PRIVATE
  GLM_ENABLE_EXPERIMENTAL
  PEGASUS_SHADER_DIR="${SHADER_OUT_DIR}"
  PEGASUS_LIBRARY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/library")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(pegasus PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
endif()

# --- Arduino firmware simulator --------------------------------------------------
# Runs arduino/pegasus_arm/pegasus_arm.ino on the PC behind a pseudo terminal
# (/tmp/pegasus-arduino-sim), so the app can be tested without a board.
add_executable(pegasus_arduino_sim arduino/sim/sim_main.cpp)
target_include_directories(pegasus_arduino_sim PRIVATE arduino/sim)
set_source_files_properties(arduino/sim/sim_main.cpp PROPERTIES OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/arduino/pegasus_arm/pegasus_arm.ino)
add_dependencies(pegasus pegasus_arduino_sim)
