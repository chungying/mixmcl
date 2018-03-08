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
    amcl_modified
    random_numbers
)

find_package(Boost REQUIRED)

INCLUDE(FindPkgConfig)
pkg_check_modules(nuklei REQUIRED nuklei)
pkg_check_modules(flann REQUIRED flann)
 
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
  ${amcl_modified_INCLUDE_DIRS}
  ${flann_INCLUDE_DIRS}
)

##Build libraries##
add_library(mcl
  src/mcl/MCL.cpp
  src/amcl/pf/pf_resample.cpp
)
target_link_libraries(mcl
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(mixmcl_node
  #definition of class MixmclNode 
  src/mixmcl/MixmclNode.cpp
)
target_link_libraries(mixmcl_node
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl dualmcl_tool
)

add_library(amcl_node
  src/amcl/AmclNode.cpp
)
target_link_libraries(amcl_node
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl
)

add_library(markov_node
  src/markov/MarkovNode.cpp
)
target_link_libraries(markov_node
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl
)

add_library(mcmcl_node
  src/mcmcl/McmclNode.cpp
)
target_link_libraries(mcmcl_node
  ${amcl_modified_LIBRARIES}
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
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  ${flann_LIBRARIES}
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
  ${flann_LIBRARIES}
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
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl markov_node
)

add_executable(kdtree_test
  src/kdtree_test.cpp
)
target_link_libraries(kdtree_test
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(amcl
  src/amcl.cpp
)
target_link_libraries(amcl
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl amcl_node
)

add_executable(mcmcl
  src/mcmcl.cpp
)
target_link_libraries(mcmcl
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  mcl mcmcl_node
)

add_executable(mixmcl
  src/mixmcl.cpp
)
target_link_libraries(mixmcl
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl mixmcl_node dualmcl_tool
)

add_executable(buildKDT
  src/buildKDT.cpp
)
target_link_libraries(buildKDT
  ${amcl_modified_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${nuklei_LIBRARIES}
  mcl dualmcl_tool
)

add_executable(dual
  src/dual.cpp
)
target_link_libraries(dual
  ${amcl_modified_LIBRARIES}
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

##  message(STATUS "amcl_LIBRARIES is ${amcl_modified_LIBRARIES}")
##  message(STATUS "RUNTIME_OUTPUT_DIRECTORY is ${RUNTIME_OUTPUT_DIRECTORY}")
##  message(STATUS "CMAKE_RUNTIME_OUTPUT_DIRECTORY is ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
##  message(STATUS "PROJECT_NAME is ${PROJECT_NAME}")
##  message(STATUS "CMAKE_INSTALL_PREFIX is ${CMAKE_INSTALL_PREFIX}")
##  message(STATUS "CATKIN_PACKAGE_SHARE_DESTINATION is ${CATKIN_PACKAGE_SHARE_DESTINATION}")
##  message(STATUS "CATKIN_PACKAGE_LIB_DESTINATION is ${CATKIN_PACKAGE_LIB_DESTINATION}")
##  message(STATUS "CATKIN_PACKAGE_BIN_DESTINATION is ${CATKIN_PACKAGE_BIN_DESTINATION}")
##  message(STATUS "CATKIN_PACKAGE_INCLUDE_DESTINATION is ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
