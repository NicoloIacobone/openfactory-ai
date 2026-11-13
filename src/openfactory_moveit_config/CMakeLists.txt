cmake_minimum_required(VERSION 3.8)
project(openfactory_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# generators of the ring configuration (ROS-free)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch srdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_srdf test/test_srdf.py)
  ament_add_pytest_test(test_ring_config test/test_ring_config.py)
endif()

ament_package()
