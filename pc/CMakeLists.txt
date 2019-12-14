cmake_minimum_required(VERSION 3.11.0)
project(pats)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1) # Save the command line compile commands in the build output
set(SRC_DIR ../src/)
set(ignoreMe "${QT_QMAKE_EXECUTABLE}") #get rid unused variable warning:
set(VIZ_3D FALSE)

# add packages
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(realsense2 REQUIRED)
find_package( OpenCV REQUIRED )
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
#SET(CMAKE_PREFIX_PATH, "/apps/gstreamer/")
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.8
 gstreamer-sdp-1.0>=1.8
 gstreamer-video-1.0>=1.8
 gstreamer-app-1.0>=1.8)
#pkg_search_module(GST REQUIRED gstreamer-1.0)
#pkg_search_module(GSTVIDEO REQUIRED gstreamer-video-1.0)
#pkg_search_module(GSTSDP REQUIRED gstreamer-sdp-1.0)
#pkg_search_module(GSTAPP REQUIRED gstreamer-app-1.0)
MESSAGE( STATUS "GST_INCLUDE_DIRS: " ${GST_INCLUDE_DIRS} )
include_directories(SYSTEM ${GST_INCLUDE_DIRS})
link_directories(SYSTEM ${GST_LIBRARY_DIRS})

include_directories(SYSTEM "${SRC_DIR}")
include_directories(SYSTEM "${SRC_DIR}/third_party")
include_directories(SYSTEM "${SRC_DIR}/trackers")
include_directories(SYSTEM "${SRC_DIR}/filtering")
include_directories(SYSTEM "${SRC_DIR}/logging")
include_directories(SYSTEM "${SRC_DIR}/cam")
include_directories(SYSTEM "${SRC_DIR}/third_party/ros_msgs/")

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


#**************QML***********
if(WITH_GUI)
 # Find includes in corresponding build directories
 set(CMAKE_INCLUDE_CURRENT_DIR ON)
 # Instruct CMake to run moc automatically when needed
 set(CMAKE_AUTOMOC ON)
 # Create code from a list of Qt designer ui files
 set(CMAKE_AUTOUIC ON)

 find_package(Qt5 COMPONENTS Widgets Qml Quick REQUIRED)
 qt5_add_resources(QT_RESOURCES ../src/gui/qml.qrc)

 FILE(GLOB GUI_SRCS "../src/gui/*.cpp" "../src/gui/*.h")

 SET(HASGUI TRUE)
 ADD_DEFINITIONS(-DHASGUI=${HASGUI})

endif()
#**************/QML***********

# sources
FILE(GLOB SRCS "../src/*.cpp" "../src/*.h" "../src/cam/*.cpp" "../src/cam/*.h" "../src/trackers/*.cpp" "../src/trackers/*.h" "../src/filtering/*.cpp" "../src/filtering/*.h" "../src/logging/*.h" "../src/logging/*.cpp" )
FILE(GLOB THIRD_PARTY_SRCS "../src/third_party/*.h" "../src/third_party/*.hpp" "../src/third_party/*.cpp" "../src/third_party/*.c" "../src/third_party/cereal/*.hpp" "../src/third_party/tinyxml/*.h" "../src/third_party/tinyxml/*.cpp" "../xml/*.xml" "../src/third_party/3dviz/*.cpp" "../src/third_party/3dviz/*.h")

if(VIZ_3D)
    include_directories("../src/third_party/3dviz/ros_msgs/")
    include_directories("../src/third_party/3dviz/publishers/")
    FILE(GLOB ROS_SRCS "../src/third_party/3dviz/ros_msgs/*.cxx" "../src/third_party/3dviz/ros_msgs/*.h" "../src/third_party/3dviz/fastrtps/*.cxx" "../src/third_party/3dviz/fastrtps/*.h" "../src/third_party/3dviz/publishers/*.cpp" "../src/third_party/3dviz/publishers/*.h")
endif()

include(CheckCXXCompilerFlag)
#-Wimplicit-fallthrough -> only supported by gcc >7 (ubuntu 18)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs -msse3 -Wall -Wextra -Wlogical-op -Wuseless-cast -Wold-style-cast -Wdouble-promotion -Wshadow -Wformat=2  -Wno-missing-field-initializers")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# build program
add_executable(${PROJECT_NAME} ${SRCS} ${THIRD_PARTY_SRCS} ${GUI_SRCS} ${QT_RESOURCES} ${ROS_SRCS})

if(WITH_GUI)
 qt5_use_modules(${PROJECT_NAME} Widgets)
 qt5_use_modules(${PROJECT_NAME} Qml)
 qt5_use_modules(${PROJECT_NAME} Quick)
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

if(WITH_GUI)
 target_link_libraries(Qt5::Quick)
 target_link_libraries(Qt5::Qml)
 target_link_libraries(Qt5::Widgets)
endif()

if(WITH_AIRSIM)
 target_link_libraries(${PROJECT_NAME} AirLib)
 target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
 target_link_libraries(${PROJECT_NAME} MavLinkCom)
endif()
