cmake_minimum_required(VERSION 2.8.3)
project(lane_detect)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

set(OpenCV_STATIC OFF)
find_package(  OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES api_lane_detection
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(3rdparty/json)

add_library(api_lane_detection
  src/detectlane.cpp
  src/carcontrol.cpp
)

add_library(refactored_lane_detection
  src/car_controller.cpp
  src/lane_detection.cpp
)

target_link_libraries(refactored_lane_detection PRIVATE nlohmann_json::nlohmann_json)

add_executable(lane_detect src/main.cpp)

add_dependencies(lane_detect ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  lane_detect
  api_lane_detection
  refactored_lane_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

install(TARGETS api_lane_detection ${ADDITIONAL_EXECUTABLES}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
