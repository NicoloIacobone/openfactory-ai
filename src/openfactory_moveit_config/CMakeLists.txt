cmake_minimum_required(VERSION 3.8)
project(openfactory_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch srdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_srdf test/test_srdf.py)
endif()

ament_package()
