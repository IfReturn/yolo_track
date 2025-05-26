cmake_minimum_required(VERSION 3.8)
project(yolo_track)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(track_node
  src/trackNode.cpp
)
ament_auto_add_executable(servo_node
  src/controlNode.cpp
)

ament_auto_package()
