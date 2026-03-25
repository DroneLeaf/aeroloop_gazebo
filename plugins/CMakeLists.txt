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

# Image bridge: subscribes to a Gazebo camera topic and writes raw frames to stdout
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(gz_image_bridge gz_image_bridge.cc)
set_property(TARGET gz_image_bridge PROPERTY CXX_STANDARD 17)
target_link_libraries(gz_image_bridge
  PRIVATE
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
)
