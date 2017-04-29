cmake_minimum_required(VERSION 2.8)
project(mu-g)

# directories
set (PROJECT_SRC_DIR ../src)

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
FILE(GLOB MY_SRC_FILES "../src/*.cpp" "../src/*.h" "../src/*.c" "../src/cereal*.hpp")

# use sse3 instruction set for libelas and c11 for threading (among others)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -std=c++11 -pthread")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -std=c++0x -pthread")

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
add_executable( MUG ${MY_SRC_FILES} )

target_link_libraries( MUG ${OpenCV_LIBS} ${GST_LIBRARIES} ${GSTSDP_LIBRARIES} ${GSTAPP_LIBRARIES})
