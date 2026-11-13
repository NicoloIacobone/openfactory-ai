cmake_minimum_required(VERSION 3.8)
project(openfactory_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/depth_sensor_model.py DESTINATION lib/${PROJECT_NAME}
        RENAME depth_sensor_model)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_depth_noise test/test_depth_noise.py)
  ament_add_pytest_test(test_ring_config test/test_ring_config.py)
endif()

ament_package()
