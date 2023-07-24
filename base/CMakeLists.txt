cmake_minimum_required(VERSION 3.10.2)
project(executor)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-DUSE_QPOASES)
remove_definitions(-DUSE_OSQP)

option(PATS_PROFILING "pats_profiling" 0)
if(PATS_PROFILING)
  add_definitions(-DPATS_PROFILING)
  message("Set -DPATS_PROFILING")
endif(PATS_PROFILING)

option(ROSVIS "rosviz" 0)
if(ROSVIS)
  add_definitions(-DROSVIS)
  message("Set -DROSVIS")
else()
  remove_definitions((-DROSVIS))
  message("Unset -DROSVIS")
endif(ROSVIS)

option(OPTI_ROSVIS "opti_rosviz" 0)
if(OPTI_ROSVIS)
  add_definitions(-DOPTI_ROSVIS)
  add_definitions(-DROSVIS)
  set(ROSVIS 1)
  message("Set -DOPTI_ROSVIS")
else()
  remove_definitions((-DOPTI_ROSVIS))
  message("Unset -DOPTI_ROSVIS")
endif(OPTI_ROSVIS)

set(SRC_DIR ./src/)
set(ignoreMe "${QT_QMAKE_EXECUTABLE}") #get rid unused variable warning:
include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs -Wall -Wextra -Wlogical-op -Wuseless-cast -Wold-style-cast -Wdouble-promotion -Wshadow -Wformat=2  -Wno-missing-field-initializers")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
MESSAGE( STATUS "CMAKE BUILD TYPE: " ${CMAKE_BUILD_TYPE} )


# sources
FILE(GLOB SRCS "${SRC_DIR}*.cpp" "${SRC_DIR}*.h" "${SRC_DIR}navigation/*.cpp" "${SRC_DIR}navigation/*.h" "${SRC_DIR}cam/*.cpp" "${SRC_DIR}cam/*.h" "${SRC_DIR}trackers/*.cpp" "${SRC_DIR}trackers/*.h" "${SRC_DIR}filtering/*.cpp" "${SRC_DIR}filtering/*.h" "${SRC_DIR}logging/*.h" "${SRC_DIR}logging/*.cpp" "${SRC_DIR}rc/*.h" "${SRC_DIR}rc/*.cpp" "${SRC_DIR}airsim/*.cpp" "${SRC_DIR}airsim/*.h" "${SRC_DIR}/flightarea/*.cpp" "${SRC_DIR}/flightarea/*.h" "${SRC_DIR}benchmark/*.cpp" "${SRC_DIR}benchmark/*.h")
FILE(GLOB THIRD_PARTY_SRCS "${SRC_DIR}third_party/*.h" "${SRC_DIR}third_party/*.hpp" "${SRC_DIR}third_party/*.cpp" "${SRC_DIR}third_party/*.c" "${SRC_DIR}third_party/cereal/*.hpp" "${SRC_DIR}third_party/tinyxml/*.h" "${SRC_DIR}third_party/tinyxml/*.cpp" "../xml/*.xml" "../xml/flightplans/*.xml" "${SRC_DIR}third_party/3dviz/*.cpp" "${SRC_DIR}third_party/3dviz/*.h")
FILE(GLOB OPTI_SRCS "${SRC_DIR}/optimization/rapid_route*")

if(ROSVIS)
  FILE(GLOB ROSVIS_SRCS "${SRC_DIR}/rosvisualization/*")
endif()

add_executable(${PROJECT_NAME} ${SRCS} ${THIRD_PARTY_SRCS} ${ROS_SRCS} ${QT_RESOURCES} ${OPTI_SRCS} ${ROSVIS_SRCS})

# add packages
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(realsense2 REQUIRED)
find_package( OpenCV 4.3.0 REQUIRED )
find_package(OpenSSL REQUIRED)

if(ROSVIS)
  find_package(roscpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
endif()

MESSAGE( STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
MESSAGE( STATUS "OPENCV_VERSION: " ${OpenCV_VERSION})
MESSAGE( STATUS "EIGEN3_INCLUDE_DIRS: " ${EIGEN3_INCLUDE_DIRS})

set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

#gstreamer:
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.8
 gstreamer-sdp-1.0>=1.8
 gstreamer-video-1.0>=1.8
 gstreamer-app-1.0>=1.8)
MESSAGE( STATUS "GST_INCLUDE_DIRS: " ${GST_INCLUDE_DIRS} )
MESSAGE( STATUS "GST_LIBRARIES: " ${GST_LIBRARIES} )
MESSAGE( STATUS "GST_LIBRARY_DIRS: " ${GST_LIBRARY_DIRS} )

#************airsim******************
if(WITH_AIRSIM)
  set(USER $ENV{USER})
  set(AIRSIM_ROOT /home/${USER}/code/AirSim)
  add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
  add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
  add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

  set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
  set(RPC_LIB rpc)
  message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

  include_directories(SYSTEM
    include
    ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/AirLib/include
    ${RPC_LIB_INCLUDES}
    ${AIRSIM_ROOT}/MavLinkCom/include
    ${AIRSIM_ROOT}/MavLinkCom/common_utils
  )
  SET(HAS_AIRSIM 1)
else()
  SET(HAS_AIRSIM 0)
endif()
ADD_DEFINITIONS(-DHAS_AIRSIM=${HAS_AIRSIM})
#/airsim*******************


target_compile_options (${PROJECT_NAME} PRIVATE  ${GST_CFLAGS})

target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}trackers")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}filtering")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}logging")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}cam")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}rc")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}airsim")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}navigation")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}flightarea")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}optimization")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}benchmark")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}third_party/ros_msgs/")
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE "${SRC_DIR}third_party")
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${GST_INCLUDE_DIRS})

if(ROSVIS)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE "${SRC_DIR}rosvisualization")
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${roscpp_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${std_msgs_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${tf2_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${tf2_ros_INCLUDE_DIRS})
endif()

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} OpenSSL::SSL)
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${GST_LIBRARIES} ${GSTSDP_LIBRARIES} ${GSTAPP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} stdc++fs) #until we upgrade to gcc 8

if(ROSVIS)
  target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES})
endif()
if(WITH_AIRSIM)
    target_link_libraries(${PROJECT_NAME} AirLib)
endif()

set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation prefix")
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/static)
