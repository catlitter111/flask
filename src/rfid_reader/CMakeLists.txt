cmake_minimum_required(VERSION 3.8)
project(rfid_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RfidTag.msg"
  "msg/RfidTagArray.msg"
  "msg/RfidReaderStatus.msg"
  "srv/RfidCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python executables
install(PROGRAMS
  rfid_reader/rfid_reader_node.py
  rfid_reader/rfid_test_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install Python package
install(DIRECTORY rfid_reader/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()