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
    std_msgs
    stamped_std_msgs
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
    cfg/MCMCL.cfg
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
  LIBRARIES mcl dualmcl_tool mixmcl_node amcl_node mcmcl_node markov_node
)

##include dirrectories##
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

##Build libraries##
add_library(mcl
  src/mcl/MCL.cpp
  src/amcl/pf/pf_resample.cpp
)
target_link_libraries(mcl
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(mixmcl_node
  #definition of class MixmclNode 
  src/mixmcl/MixmclNode.cpp
)
target_link_libraries(mixmcl_node
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl dualmcl_tool
)

add_library(amcl_node
  src/amcl/AmclNode.cpp
)
target_link_libraries(amcl_node
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl
)

add_library(markov_node
  src/markov/MarkovNode.cpp
)
target_link_libraries(markov_node
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl
)

add_library(mcmcl_node
  src/mcmcl/McmclNode.cpp
)
target_link_libraries(mcmcl_node
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl mixmcl_node
)

add_library(dualmcl_tool
  #IO definition for recording robot poses and laser features
  src/io/paramio.cpp
  src/io/dataio.cpp
  #definition of class SamplingNode for collecting sampling pose and laser features
  src/mixmcl/SamplingNode.cpp
  #definition of class KCGrid for build density trees
  src/mixmcl/KCGrid.cpp
)
target_link_libraries(dualmcl_tool
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  mcl
)

##Build executables##
add_executable(iotest
  src/iotest.cpp
)
target_link_libraries(iotest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${nuklei_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  dualmcl_tool
)

add_executable(roscheck
  src/roscheck.cpp
)
target_link_libraries(roscheck
  ${catkin_LIBRARIES}
)

add_executable(markov
  src/markov.cpp
)
target_link_libraries(markov
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl markov_node
)

add_executable(amcl
  src/amcl.cpp
)
target_link_libraries(amcl
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl amcl_node
)

add_executable(mcmcl
  src/mcmcl.cpp
)
target_link_libraries(mcmcl
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl mcmcl_node
)

add_executable(mixmcl
  src/mixmcl.cpp
)
target_link_libraries(mixmcl
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl mixmcl_node dualmcl_tool
)

add_executable(buildKDT
  src/buildKDT.cpp
)
target_link_libraries(buildKDT
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl dualmcl_tool
)

add_executable(dual
  src/dual.cpp
)
target_link_libraries(dual
  ${amcl_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl mixmcl_node dualmcl_tool
)

add_executable(multi_array_test
  src/multi_array_test.cpp
)
target_link_libraries(multi_array_test
  ${catkin_LIBRARIES}
)

install( TARGETS
    mixmcl amcl mcmcl buildKDT iotest roscheck dual markov
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install( TARGETS
    mcl dualmcl_tool 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

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
