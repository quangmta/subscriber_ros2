cmake_minimum_required(VERSION 3.5)
project(data_subscriber)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(image_subscriber src/image_subscriber.cpp)
target_include_directories(image_subscriber PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_subscriber ${OpenCV_LIBRARIES})
ament_target_dependencies(image_subscriber rclcpp sensor_msgs)

add_executable(depth_image_subscriber src/depth_image_subscriber.cpp)
target_include_directories(depth_image_subscriber PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_image_subscriber ${OpenCV_LIBRARIES})
ament_target_dependencies(depth_image_subscriber rclcpp sensor_msgs)

add_executable(laser_scan_subscriber src/laser_scan_subscriber.cpp)
ament_target_dependencies(laser_scan_subscriber rclcpp sensor_msgs)


add_library(DataSubscriber
  src/data_subscriber.cpp
)
ament_target_dependencies(DataSubscriber
  rclcpp
  sensor_msgs
  geometry_msgs
  OpenCV
  image_geometry
  nav_msgs
  tf2
  tf2_geometry_msgs
)
target_include_directories(DataSubscriber PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(data_subscriber_node src/data_main.cpp)

target_link_libraries(data_subscriber_node DataSubscriber)

install(TARGETS
        image_subscriber
        depth_image_subscriber
        laser_scan_subscriber
        data_subscriber_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

        install(DIRECTORY include
        DESTINATION include/
      )    

install(TARGETS DataSubscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
