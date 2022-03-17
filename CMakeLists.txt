cmake_minimum_required(VERSION 3.8)
project(perception_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)

include(FindProtobuf REQUIRED)
ament_auto_find_build_dependencies()

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
)

install(TARGETS image_subscriber_component
  EXPORT export_image_subscriber_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(
  FILES ${PROTO_HDRS}
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
