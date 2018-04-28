cmake_minimum_required(VERSION 3.1.0)
project(pats)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(REALSENSE realsense2)
# add packages
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
include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})

set (CEREAL_SRC_DIR ../src/)
include_directories("${CEREAL_SRC_DIR}")

# sources
FILE(GLOB SRCS "../src/*.cpp" "../src/*.h" "../src/*.c" "../src/cereal*.hpp")

# use sse3 instruction set for libelas and c11 for threading (among others)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse3 -pthread")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -msse3 -pthread")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

#detect whether we build for ARM drone or pc
IF (NOT DEFINED _PC)
    SET(_PC TRUE)
ENDIF()
ADD_DEFINITIONS(-D_PC=${_PC})

IF(CMAKE_BUILD_TYPE STREQUAL Debug)
        set(DEBUG_FLAG 1)
        ADD_DEFINITIONS(-DDEBUG_FLAG=${DEBUG_FLAG})
        MESSAGE( STATUS "DEBUG MODE" )
ENDIF(CMAKE_BUILD_TYPE STREQUAL Debug)

# build program
add_executable(pats ${SRCS})

target_link_libraries( pats ${OpenCV_LIBS} ${REALSENSE} ${GST_LIBRARIES} ${GSTSDP_LIBRARIES} ${GSTAPP_LIBRARIES})

