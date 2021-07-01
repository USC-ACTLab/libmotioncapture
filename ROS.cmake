cmake_minimum_required(VERSION 2.8.3)
project(libmotioncapture)

find_package(catkin)

set(ENABLE_QUALISYS True CACHE STRING "Enable Qualisys")
set(ENABLE_OPTITRACK True CACHE STRING "Enable Optitrack")
set(ENABLE_VICON True CACHE STRING "Enable Vicon")
set(ENABLE_PHASESPACE False CACHE STRING "Enable Phasespace")
set(ENABLE_VRPN False CACHE STRING "Enable VRPN")

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libmotioncapture
  CATKIN_DEPENDS
  DEPENDS
)

find_package(PCL REQUIRED)
set(VICON_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/vicon-datastream-sdk/)
set(NATNET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/NatNetLinux/)
set(PHASESPACE_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/phasespace_sdk/)
set(QUALISYS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/qualisys_cpp_sdk/)

###########
## Build ##
###########

## Additional include folders
set(my_include_directories
  include
  ${PCL_INCLUDE_DIRS}
)
set(my_link_directories)
set(my_files
  src/motioncapture.cpp
  src/testmocap.cpp
)
set(my_libraries
  ${PCL_LIBRARIES}
)

if (ENABLE_VICON)
  add_subdirectory(externalDependencies/vicon-datastream-sdk)
  set(my_include_directories
    ${my_include_directories}
    ${VICON_SDK_DIR}/Vicon/CrossMarket/DataStream
  )
  set(my_files
    ${my_files}
    src/vicon.cpp
  )
  set(my_libraries
    ${my_libraries}
    ViconDataStreamSDK_CPP
  )
endif()

if (ENABLE_OPTITRACK)
  set(my_include_directories
    ${my_include_directories}
    ${NATNET_DIR}/include
  )
  set(my_files
    ${my_files}
    src/optitrack.cpp
  )
endif()

if (ENABLE_PHASESPACE)
  set(my_include_directories
    ${my_include_directories}
    ${PHASESPACE_SDK_DIR}/include
  )
  set(my_link_directories
    ${my_link_directories}
    ${PHASESPACE_SDK_DIR}/lib64
  )
  set(my_files
    ${my_files}
    src/phasespace.cpp
  )
  set(my_libraries
    ${my_libraries}
    owlsock
    OWL
  )
endif()

if (ENABLE_QUALISYS)
  add_subdirectory(externalDependencies/qualisys_cpp_sdk)
  set(my_include_directories
    ${my_include_directories}
    ${QUALISYS_DIR}
  )
  set(my_files
    ${my_files}
    src/qualisys.cpp
  )
  set(my_libraries
    ${my_libraries}
    qualisys_cpp_sdk
  )
endif()

if (ENABLE_VRPN)
  find_package(VRPN REQUIRED)
  set(my_include_directories
    ${my_include_directories}
    ${VRPN_INCLUDE_DIR}
  )
  set(my_files
    ${my_files}
    src/vrpn.cpp
  )
  set(my_libraries
    ${my_libraries}
    ${VRPN_LIBRARIES}
  )
endif()


include_directories(
  ${my_include_directories}
)

link_directories(
  ${my_link_directories}
)

## Declare a cpp library
add_library(libmotioncapture
  ${my_files}
)

## Specify libraries to link a library or executable target against
target_link_libraries(libmotioncapture
  ${my_libraries}
)
set(LIBMOTIONCAPTURE_LINK_DIR ${my_link_directories} CACHE STRING "link directories for libmotioncapture")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS crazyflie crazyflie_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_crazyflie.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

