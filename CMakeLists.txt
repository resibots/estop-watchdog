cmake_minimum_required(VERSION 2.8.3)
project(estop_trigger)

# Tell CMake where to find "FindXXX.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    estop_gateway_udp
)

catkin_package(
  CATKIN_DEPENDS
    estop_gateway_udp
  INCLUDE_DIRS
    include
)

find_package(OpenSSL REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIRS}
)

add_executable(trigger_test src/trigger_test.cpp)
target_link_libraries(trigger_test
  ${catkin_LIBRARIES}
  ${OPENSSL_LIBRARIES}
)
add_dependencies(trigger_test estop_gateway_udp_generate_messages_cpp)

add_executable(estop_trigger src/estop_trigger_sample.cpp src/estop_trigger.cpp)
target_link_libraries(estop_trigger
  ${catkin_LIBRARIES}
  ${OPENSSL_LIBRARIES}
)
add_dependencies(estop_trigger estop_gateway_udp_generate_messages_cpp)

add_executable(ros_control_trigger
  src/ros_control_trigger.cpp
  src/estop_trigger.cpp)
target_link_libraries(ros_control_trigger
  ${catkin_LIBRARIES}
  ${OPENSSL_LIBRARIES}
)
add_dependencies(ros_control_trigger estop_gateway_udp_generate_messages_cpp)

# install(TARGETS hexapod_ik
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
