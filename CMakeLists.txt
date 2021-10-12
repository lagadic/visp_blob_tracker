cmake_minimum_required(VERSION 3.0)
project(visp_blobs_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  visp_bridge
)
# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP)

find_package(Boost REQUIRED
  COMPONENTS filesystem system signals regex date_time program_options thread)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
#link_directories(
#  ${BULLET_LIBRARY_DIRS}
#  ${Boost_LIBRARY_DIRS}
#  ${VISP_LIBRARY_DIRS}
#  ${catkin_LIBRARY_DIRS}
#  )

catkin_package(
   LIBRARIES 
   vpBlobsTargetTracker
   vpColorDetection

   CATKIN_DEPENDS
   geometry_msgs
   message_filters
   roscpp
   sensor_msgs
   std_msgs

   DEPENDS
   VISP
   )

# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)


# Add ViSP definitions.
add_definitions(${VISP_DEFINITIONS})

# Declare a cpp library
add_library(vpBlobsTargetTracker
  src/vpBlobsTargetTracker.cpp
  src/vpBlobsTargetTracker.h
)
target_link_libraries(vpBlobsTargetTracker
  ${VISP_LIBRARIES}
)

# Declare a cpp library
add_library(vpColorDetection
  src/vpColorDetection.cpp
  src/vpColorDetection.h
)
target_link_libraries(vpColorDetection
  ${VISP_LIBRARIES}
)



add_executable(visp_blobs_tracker
  src/main.cpp
  src/names.cpp
  src/names.h
  src/node.cpp
  src/node.h
  )

  target_link_libraries(visp_blobs_tracker
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    vpBlobsTargetTracker
    vpColorDetection)



