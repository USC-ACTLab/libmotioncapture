cmake_minimum_required(VERSION 2.8.3)
project(libmotioncapture)

set(ENABLE_QUALISYS True CACHE STRING "Enable Qualisys")
set(ENABLE_OPTITRACK True CACHE STRING "Enable Optitrack")
set(ENABLE_VICON True CACHE STRING "Enable Vicon")
set(ENABLE_PHASESPACE False CACHE STRING "Enable Phasespace")
set(ENABLE_VRPN False CACHE STRING "Enable VRPN")

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

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
