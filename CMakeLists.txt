cmake_minimum_required(VERSION 3.1.2)
project(u2dar_receiver)

FILE(GLOB_RECURSE HEADER_FILES include *.h)
FILE(GLOB_RECURSE LAUNCHER_FILES include *.launch)
FILE(GLOB_RECURSE YAML_FILES include *.yaml)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -Wall -Wextra -Wno-psabi")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -std=c++17 -g -Wall -Wextra -Wno-psabi -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  u2dar_receiver_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs u2dar_receiver_msgs
)

add_library(rpi5_periph include/rpi5_periph/rp1.c include/rpi5_periph/i2c.c include/rpi5_periph/spi.c)
add_library(max9939 include/max9939/max9939.c)
add_library(max262 include/max262/max262.c)
add_library(alsa_pcm1822 include/alsa_pcm1822/alsa_pcm1822.c)
add_library(sensors include/sensors/aht20.c include/sensors/bmp280.c)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(recorder_node src/recorder_node.cpp)

target_link_libraries(recorder_node ${catkin_LIBRARIES} rpi5_periph sensors max9939 max262 alsa_pcm1822 asound m pthread)

install(TARGETS recorder_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
