cmake_minimum_required(VERSION 3.11.0)
project(pats)

set(SRC_DIR ../src/)
set(ignoreMe "${QT_QMAKE_EXECUTABLE}") #get rid unused variable warning:
include(CheckCXXCompilerFlag)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs -msse3 -Wall -Wextra -Wlogical-op -Wuseless-cast -Wold-style-cast -Wdouble-promotion -Wshadow -Wformat=2  -Wno-missing-field-initializers")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# sources
FILE(GLOB SRCS "${SRC_DIR}*.cpp" "${SRC_DIR}*.h" "${SRC_DIR}navigation/*.cpp" "${SRC_DIR}navigation/*.h" "${SRC_DIR}cam/*.cpp" "${SRC_DIR}cam/*.h" "${SRC_DIR}trackers/*.cpp" "${SRC_DIR}trackers/*.h" "${SRC_DIR}filtering/*.cpp" "${SRC_DIR}filtering/*.h" "${SRC_DIR}logging/*.h" "${SRC_DIR}logging/*.cpp" )
FILE(GLOB THIRD_PARTY_SRCS "${SRC_DIR}third_party/*.h" "${SRC_DIR}third_party/*.hpp" "${SRC_DIR}third_party/*.cpp" "${SRC_DIR}third_party/*.c" "${SRC_DIR}third_party/cereal/*.hpp" "${SRC_DIR}third_party/tinyxml/*.h" "${SRC_DIR}third_party/tinyxml/*.cpp" "../xml/*.xml" "../xml/flightplans/*.xml" "${SRC_DIR}third_party/3dviz/*.cpp" "${SRC_DIR}third_party/3dviz/*.h")

if(VIZ_3D)
    FILE(GLOB ROS_SRCS "${SRC_DIR}third_party/3dviz/ros_msgs/*.cxx" "${SRC_DIR}third_party/3dviz/ros_msgs/*.h" "${SRC_DIR}third_party/3dviz/fastrtps/*.cxx" "${SRC_DIR}third_party/3dviz/fastrtps/*.h" "${SRC_DIR}third_party/3dviz/publishers/*.cpp" "${SRC_DIR}third_party/3dviz/publishers/*.h")
endif()

add_executable(${PROJECT_NAME} ${SRCS} ${THIRD_PARTY_SRCS} ${ROS_SRCS} ${QT_RESOURCES})

# add packages
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(realsense2 REQUIRED)
find_package( OpenCV 4.3.0 REQUIRED )
MESSAGE( STATUS "OpenCV_DIR: " ${OpenCV_DIR} )
MESSAGE( STATUS "OpenCV_LIBS: " ${OpenCV_LIBS} )
MESSAGE( STATUS "OpenCV_LIB_DIR: " ${OpenCV_LIB_DIR} )
MESSAGE( STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS} )
MESSAGE( STATUS "OpenCV_COMPUTE_CAPABILITIES: " ${OpenCV_COMPUTE_CAPABILITIES} )
MESSAGE( STATUS "OpenCV_SHARED: " ${OpenCV_SHARED} )
MESSAGE( STATUS "OpenCV_CONFIG_PATH: " ${OpenCV_CONFIG_PATH} )
MESSAGE( STATUS "OpenCV_INSTALL_PATH: " ${OpenCV_INSTALL_PATH} )
MESSAGE( STATUS "OPENCV_VERSION: " ${OpenCV_VERSION} )

if(VIZ_3D)
    add_definitions(-DVIZ_3D)
    list(APPEND CMAKE_PREFIX_PATH /opt/ros/dashing/share/fastcdr/cmake)
    find_package(fastcdr REQUIRED)
    find_package(fastrtps REQUIRED)
endif()

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


target_compile_options (${PROJECT_NAME} PRIVATE  ${GST_CFLAGS})

target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}/trackers")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}/filtering")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}/logging")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}/cam")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}/navigation")
target_include_directories(${PROJECT_NAME} PRIVATE "${SRC_DIR}/third_party/ros_msgs/")
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE "${SRC_DIR}/third_party")
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${GST_INCLUDE_DIRS})
if(VIZ_3D)
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE "${SRC_DIR}third_party/3dviz/ros_msgs/")
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE "${SRC_DIR}third_party/3dviz/publishers/")
endif()
if(WITH_AIRSIM)
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${AIRSIM_ROOT}/AirLib/deps/eigen3)
endif()

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${GST_LIBRARIES} ${GSTSDP_LIBRARIES} ${GSTAPP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} stdc++fs) #until we upgrade to gcc 8
target_link_libraries(${PROJECT_NAME} ${FASTRTPS_LIBRARIES})
if(VIZ_3D)
    target_link_libraries(${PROJECT_NAME} fastrtps)
endif()
if(WITH_AIRSIM)
    target_link_libraries(${PROJECT_NAME} AirLib)
    target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
    target_link_libraries(${PROJECT_NAME} MavLinkCom)
endif()
