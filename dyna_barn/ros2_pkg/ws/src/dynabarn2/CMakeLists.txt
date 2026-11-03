cmake_minimum_required(VERSION 3.16)
project(dynabarn2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)

# Gazebo system plugin that moves obstacles along timed waypoints
add_library(DynaBarnObstacleMover SHARED src/ObstacleMover.cc)
target_link_libraries(DynaBarnObstacleMover
  gz-sim::gz-sim
  gz-plugin::register
  gz-math::gz-math
)

install(TARGETS DynaBarnObstacleMover
  LIBRARY DESTINATION lib
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
