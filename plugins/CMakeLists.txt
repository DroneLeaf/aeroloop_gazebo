cmake_minimum_required(VERSION 3.10.2)
project(BetaflightPlugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)

add_library(BetaflightPlugin SHARED BetaflightPlugin.cc)
set_property(TARGET BetaflightPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(BetaflightPlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-math7::gz-math7
)

add_library(ExternalPosePlugin SHARED ExternalPosePlugin.cc)
set_property(TARGET ExternalPosePlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(ExternalPosePlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-math7::gz-math7
)

add_library(RotorVisualPlugin SHARED RotorVisualPlugin.cc)
set_property(TARGET RotorVisualPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(RotorVisualPlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-math7::gz-math7
)

add_library(ViscousDragPlugin SHARED ViscousDragPlugin.cc)
set_property(TARGET ViscousDragPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(ViscousDragPlugin
  PRIVATE
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
    gz-math7::gz-math7
)

# Image bridge: subscribes to a Gazebo camera topic and writes raw frames to stdout
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# SDL2 for zero-latency direct display mode (--display flag)
find_package(SDL2 QUIET)

add_executable(gz_image_bridge gz_image_bridge.cc)
set_property(TARGET gz_image_bridge PROPERTY CXX_STANDARD 17)
target_link_libraries(gz_image_bridge
  PRIVATE
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    pthread
    rt
)

if(SDL2_FOUND)
  target_compile_definitions(gz_image_bridge PRIVATE HAS_SDL2)
  target_link_libraries(gz_image_bridge PRIVATE SDL2::SDL2)
  message(STATUS "SDL2 found — gz_image_bridge --display mode enabled")
else()
  message(STATUS "SDL2 not found — gz_image_bridge --display mode disabled (install libsdl2-dev)")
endif()
