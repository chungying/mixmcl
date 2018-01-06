cmake_minimum_required(VERSION 2.8.3)
project(mixmcl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    message_filters
    rosbag
    roscpp
    std_srvs
    tf
    tf2
    tf2_ros
    dynamic_reconfigure
    nav_msgs
    gazebo_msgs
    amcl
    random_numbers
)

find_package(Boost REQUIRED)

find_package(PCL REQUIRED
COMPONENTS
  common
  io
)

INCLUDE(FindPkgConfig)
pkg_check_modules(nuklei nuklei)
 
# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MIXMCL.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rosbag
    roscpp
    dynamic_reconfigure
    tf
    nav_msgs
    std_srvs
  INCLUDE_DIRS include
  LIBRARIES mcl mixmcl_tool
)

##include dirrectories
include_directories(
  include src
)
include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${nuklei_INCLUDE_DIRS}
  ${amcl_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(mcl
  src/mcl/mcl.cpp
)

target_link_libraries(mcl
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(mixmcl_tool
##definition of class MixmclNode 
  src/mixmcl/mixmcl.cpp
##IO definition for recording robot poses and laser features
  src/io/paramio.cpp
  src/io/dataio.cpp
##definition of class SamplingNode for collecting sampling pose and laser features
  src/mixmcl/sampling.cpp
##definition of class KCGrid for build density trees
  src/mixmcl/kcgrid.cpp
)

target_link_libraries(mixmcl_tool
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl
)

add_executable(pcd_write_test 
  src/pcd_write.cpp)
target_link_libraries(pcd_write_test
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

add_executable(iotest
  src/iotest.cpp
)

target_link_libraries(iotest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${nuklei_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  mixmcl_tool
)

add_executable(bo
  src/boostoptional.cpp
)

target_link_libraries(bo
  ${catkin_LIBRARIES}
  ${BOOST_LIBRARIES}
)

## roscheck for shell scripts
add_executable(roscheck
  src/roscheck.cpp
)

target_link_libraries(roscheck
  ${catkin_LIBRARIES}
)

## amcl_node
add_executable(amcl
  src/amcl_node.cpp
  src/amcl/amcl.cpp
)

target_link_libraries(amcl
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl
)
 
## mixmcl_node
add_executable(${PROJECT_NAME}
  src/mixmcl_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl mixmcl_tool
)

## buildKDT
add_executable(buildKDT
  src/buildKDT.cpp
)

target_link_libraries(buildKDT
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl mixmcl_tool
)

install( TARGETS
    ${PROJECT_NAME} amcl buildKDT iotest pcd_write_test roscheck bo
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( TARGETS
    mcl mixmcl_tool
    LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#message(STATUS "CATKIN_PACKAGE_INCLUDE_DESTINATION is ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
install(DIRECTORY include 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#message(STATUS "CATKIN_PACKAGE_SHARE_DESTINATION is ${CATKIN_PACKAGE_SHARE_DESTINATION}")
#install(DIRECTORY data
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
#)

##  message(STATUS "amcl_LIBRARIES is ${amcl_LIBRARIES}")
##  message(STATUS "RUNTIME_OUTPUT_DIRECTORY is ${RUNTIME_OUTPUT_DIRECTORY}")
##  message(STATUS "CMAKE_RUNTIME_OUTPUT_DIRECTORY is ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
##  message(STATUS "PROJECT_NAME is ${PROJECT_NAME}")
##  message(STATUS "CMAKE_INSTALL_PREFIX is ${CMAKE_INSTALL_PREFIX}")
##  message(STATUS "CATKIN_PACKAGE_SHARE_DESTINATION is ${CATKIN_PACKAGE_SHARE_DESTINATION}")
##  message(STATUS "CATKIN_PACKAGE_LIB_DESTINATION is ${CATKIN_PACKAGE_LIB_DESTINATION}")
##  message(STATUS "CATKIN_PACKAGE_BIN_DESTINATION is ${CATKIN_PACKAGE_BIN_DESTINATION}")
##  message(STATUS "CATKIN_PACKAGE_INCLUDE_DESTINATION is ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")

## Configure Tests
##if(CATKIN_ENABLE_TESTING)
if(FALSE)
  find_package(rostest REQUIRED)

  # Bags
  catkin_download_test_data(${PROJECT_NAME}_basic_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/basic_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 41fe43af189ec71e5e48eb9ed661a655)
  catkin_download_test_data(${PROJECT_NAME}_global_localization_stage_indexed.bag
    http://download.ros.org/data/amcl/global_localization_stage_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 752f711cf4f6e8d1d660675e2da096b0)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_prf_indexed.bag
    http://download.ros.org/data/amcl/small_loop_prf_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 e4ef0fc006872b43f12ed8a7ce7dcd81)
  catkin_download_test_data(${PROJECT_NAME}_small_loop_crazy_driving_prg_indexed.bag
    http://download.ros.org/data/amcl/small_loop_crazy_driving_prg_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 4a58d1a7962914009d99000d06e5939c)
  catkin_download_test_data(${PROJECT_NAME}_texas_greenroom_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_greenroom_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 6e3432115cccdca1247f6c807038e13d)
  catkin_download_test_data(${PROJECT_NAME}_texas_willow_hallway_loop_indexed.bag
    http://download.ros.org/data/amcl/texas_willow_hallway_loop_indexed.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 27deb742fdcd3af44cf446f39f2688a8)
  catkin_download_test_data(${PROJECT_NAME}_rosie_localization_stage.bag
    http://download.ros.org/data/amcl/rosie_localization_stage.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 3347bf3835724cfa45e958c5c1846066)

  # Maps
  catkin_download_test_data(${PROJECT_NAME}_willow-full.pgm
    http://download.ros.org/data/amcl/willow-full.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b84465cdbbfe3e2fb9eb4579e0bcaf0e)
  catkin_download_test_data(${PROJECT_NAME}_willow-full-0.05.pgm
    http://download.ros.org/data/amcl/willow-full-0.05.pgm
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test
    MD5 b61694296e08965096c5e78611fd9765)

  # Tests
  add_rostest(test/set_initial_pose.xml ARGS share_test_dir:=${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
##  add_rostest(test/set_initial_pose_delayed.xml)
##  add_rostest(test/basic_localization_stage.xml)
##  add_rostest(test/small_loop_prf.xml)
##  add_rostest(test/small_loop_crazy_driving_prg.xml)
##  add_rostest(test/texas_greenroom_loop.xml)
##  add_rostest(test/rosie_multilaser.xml)
##  add_rostest(test/texas_willow_hallway_loop.xml)

# Not sure when or if this actually passed.
#
# The point of this is that you start with an even probability
# distribution over the whole map and the robot localizes itself after
# some number of iterations of sensing and motion.
#
#  add_rostest(test/global_localization_stage.xml)
endif()
