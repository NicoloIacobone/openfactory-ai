cmake_minimum_required(VERSION 3.8)
project(openfactory_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Gazebo Harmonic through the ROS vendor packages: the cube mechanism is a system plugin
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Mechanism of the Rubik's cube model (phase 7): loaded by Gazebo with the model
add_library(RubikCubeSystem SHARED src/rubik_cube_system.cc)
target_compile_features(RubikCubeSystem PUBLIC cxx_std_17)
target_link_libraries(RubikCubeSystem PRIVATE gz-sim::gz-sim gz-plugin::register)
install(TARGETS RubikCubeSystem LIBRARY DESTINATION lib)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.dsv.in")

install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/depth_sensor_model.py DESTINATION lib/${PROJECT_NAME}
        RENAME depth_sensor_model)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_depth_noise test/test_depth_noise.py)
  ament_add_pytest_test(test_ring_config test/test_ring_config.py)
endif()

ament_package()
