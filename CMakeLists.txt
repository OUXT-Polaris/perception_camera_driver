cmake_minimum_required(VERSION 3.8)
project(perception_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)
include(FindProtobuf REQUIRED)

file(DOWNLOAD 
  https://raw.githubusercontent.com/OUXT-Polaris/perception_camera_app/master/proto/perception_camera_app.proto
  ${CMAKE_BINARY_DIR}/perception_camera_app.proto)

set(PROTO_FILES
  ${CMAKE_BINARY_DIR}/perception_camera_app.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
message( "Generate protobuf headers ... : " ${PROTO_HDRS})
message( "Generate protobuf sources ... : " ${PROTO_SRCS})
include_directories(
  include
  ${CMAKE_BINARY_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(proto SHARED
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)
target_link_libraries(proto
  ${PROTOBUF_LIBRARY}
  ${OpenCV_LIBS}
)

ament_auto_add_library(image_subscriber_component SHARED
  src/image_subscriber_component.cpp
  src/subscriber.cpp
  src/endpoint.cpp
  src/conversion.cpp
)

ament_auto_add_executable(perception_camera_driver_node
  src/perception_camera_driver_node.cpp
)

target_link_libraries(image_subscriber_component
  ${PROTOBUF_LIBRARY}
  proto
)

target_link_libraries(perception_camera_driver_node
  ${PROTOBUF_LIBRARY}
  image_subscriber_component
  glog
  proto
)

install(TARGETS image_subscriber_component
  EXPORT export_image_subscriber_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

rclcpp_components_register_nodes(image_subscriber_component
  "perception_camera_driver::ImageSubscriberComponent")

install(TARGETS proto
  EXPORT export_proto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(TARGETS
  perception_camera_driver_node
  DESTINATION lib/perception_camera_driver
)

install(
  FILES ${PROTO_HDRS}
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
