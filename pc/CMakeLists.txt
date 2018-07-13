cmake_minimum_required(VERSION 3.9.0)
project(pats)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(REALSENSE realsense2)
# add packages
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package( OpenCV REQUIRED )
#find_package( GStreamer REQUIRED )
MESSAGE( STATUS "OpenCV_DIR: " ${OpenCV_DIR} )
MESSAGE( STATUS "OpenCV_LIBS: " ${OpenCV_LIBS} )
MESSAGE( STATUS "OpenCV_LIB_DIR: " ${OpenCV_LIB_DIR} )
MESSAGE( STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS} )
MESSAGE( STATUS "OpenCV_COMPUTE_CAPABILITIES: " ${OpenCV_COMPUTE_CAPABILITIES} )
MESSAGE( STATUS "OpenCV_SHARED: " ${OpenCV_SHARED} )
MESSAGE( STATUS "OpenCV_CONFIG_PATH: " ${OpenCV_CONFIG_PATH} )
MESSAGE( STATUS "OpenCV_INSTALL_PATH: " ${OpenCV_INSTALL_PATH} )
MESSAGE( STATUS "OPENCV_VERSION: " ${OpenCV_VERSION} )

#gstreamer:
pkg_search_module(GST REQUIRED gstreamer-1.0)
pkg_search_module(GSTVIDEO REQUIRED gstreamer-video-1.0)
pkg_search_module(GSTSDP REQUIRED gstreamer-sdp-1.0)
pkg_search_module(GSTAPP REQUIRED gstreamer-app-1.0)
include_directories(SYSTEM ${GST_INCLUDE_DIRS})
link_directories(SYSTEM ${GST_LIBRARY_DIRS})

set (CEREAL_SRC_DIR ../src/)
include_directories(SYSTEM "${CEREAL_SRC_DIR}")



#************airsim******************
if(WITH_AIRSIM)
 set (AIRSIM_ROOT /home/houjebek/code/AirSim)

 add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
 add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
 add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

 LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake")
 LIST(APPEND CMAKE_MODULE_PATH "${AIRSIM_ROOT}/cmake/cmake-modules")
 INCLUDE("${AIRSIM_ROOT}/cmake/cmake-modules/CommonSetup.cmake")

 #setup include and lib for rpclib which will be referenced by other projects
 set(RPCLIB_VERSION_FOLDER rpclib-2.2.1)
 set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/${RPCLIB_VERSION_FOLDER}/include")
 #name of .a file with lib prefix
 set(RPC_LIB rpc)
 message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

 include_directories(SYSTEM ${AIRSIM_ROOT}/AirLib/deps/eigen3)

 set(CMAKE_CXX_FLAGS "\
  -ggdb -fdiagnostics-show-option \
  ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")
 set(CXX_EXP_LIB "-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")

 include_directories(
  SYSTEM ${AIRSIM_ROOT}/../airsim-test
  SYSTEM ${AIRSIM_ROOT}/AirLib/deps/eigen3
  SYSTEM ${AIRSIM_ROOT}/AirLib/include
  SYSTEM ${RPC_LIB_INCLUDES}
  SYSTEM ${AIRSIM_ROOT}/MavLinkCom/include
  SYSTEM ${AIRSIM_ROOT}/MavLinkCom/common_utils
  )

 link_directories(${AIRSIM_ROOT}/build_debug/output/lib/)
endif()
#/airsim*******************


# sources
FILE(GLOB SRCS "../src/*.cpp" "../src/*.h" "../src/*.c" "../src/cereal*.hpp")

# use sse3 instruction set for libelas and c11 for threading (among others)
include(CheckCXXCompilerFlag)
#-Wold-style-cast
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -Wall -Wextra -Wlogical-op -Wuseless-cast -Wdouble-promotion -Wshadow -Wformat=2 -Wno-missing-field-initializers")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWER)
IF(CMAKE_BUILD_TYPE_LOWER STREQUAL debug)
set(DEBUG_FLAG 1)
ADD_DEFINITIONS(-DDEBUG_FLAG=${DEBUG_FLAG})
MESSAGE( STATUS "DEBUG MODE" )
ENDIF(CMAKE_BUILD_TYPE_LOWER STREQUAL debug)

#detect whether we build for ARM drone or pc
IF (NOT DEFINED _PC)
SET(_PC TRUE)
ENDIF()
ADD_DEFINITIONS(-D_PC=${_PC})

# build program
add_executable(pats ${SRCS})


target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} ${REALSENSE} ${GST_LIBRARIES} ${GSTSDP_LIBRARIES} ${GSTAPP_LIBRARIES})
if(WITH_AIRSIM)
 target_link_libraries(${PROJECT_NAME} AirLib)
 target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
 target_link_libraries(${PROJECT_NAME} MavLinkCom)
endif()
