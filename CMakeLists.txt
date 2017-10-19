cmake_minimum_required(VERSION 2.8.3)
project(estop_watchdog)

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

add_executable(watchdog_test src/watchdog_test.cpp)
target_link_libraries(watchdog_test
  ${catkin_LIBRARIES}
  ${OPENSSL_LIBRARIES}
)
add_dependencies(watchdog_test estop_gateway_udp_generate_messages_cpp)

# install(TARGETS hexapod_ik
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
