cmake_minimum_required(VERSION 3.16)
project(dynabarn2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Gazebo system plugin that moves obstacles along timed waypoints
add_library(DynaBarnObstacleMover SHARED src/ObstacleMover.cc)
target_link_libraries(DynaBarnObstacleMover
  gz-sim::gz-sim
  gz-plugin::register
  gz-math::gz-math
)

# MPPI critic that scores trajectories against predicted obstacle positions
add_library(dynabarn_prediction_critic SHARED src/PredictionCritic.cc)
target_compile_options(dynabarn_prediction_critic PRIVATE -O3)
ament_target_dependencies(dynabarn_prediction_critic nav2_mppi_controller pluginlib rclcpp std_msgs)
pluginlib_export_plugin_description_file(nav2_mppi_controller prediction_critic.xml)

install(TARGETS DynaBarnObstacleMover dynabarn_prediction_critic
  LIBRARY DESTINATION lib
)

install(DIRECTORY worlds data config launch
  DESTINATION share/${PROJECT_NAME}
)

# World generators (ros2 run dynabarn2 generate_movers.py ...)
install(PROGRAMS
  scripts/generate_movers.py
  scripts/generate_polynomial.py
  scripts/generate_dynabarn2.py
  scripts/convert_dynabarn_classic.py
  scripts/trial_runner.py
  scripts/obstacle_predictor.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_python_install_package(dynabarn2_worlds)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
