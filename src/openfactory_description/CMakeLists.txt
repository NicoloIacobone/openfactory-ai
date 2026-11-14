cmake_minimum_required(VERSION 3.8)
project(openfactory_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ring layout and names (ROS-free), imported by the other packages
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
  ament_add_pytest_test(test_ring test/test_ring.py)
  ament_add_pytest_test(test_ring_urdf test/test_ring_urdf.py)
  ament_add_pytest_test(test_rubik_urdf test/test_rubik_urdf.py)
endif()

ament_package()
