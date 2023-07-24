cmake_minimum_required(VERSION 3.11.0)
project(pats-unittest)
find_package( OpenCV REQUIRED )
find_package(PkgConfig REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs -Wall -Wextra -Wlogical-op -Wuseless-cast -Wold-style-cast -Wdouble-promotion -Wshadow -Wformat=2  -Wno-missing-field-initializers")
MESSAGE( STATUS "CMAKE BUILD TYPE: " ${CMAKE_BUILD_TYPE} )

option(OPTI_ROSVIS "opti_rosviz" 0)
if(OPTI_ROSVIS)
  add_definitions(-DOPTI_ROSVIS)
  add_definitions(-DROSVIS)
  message("Set -DOPTI_ROSVIS")
else()
  remove_definitions((-DOPTI_ROSVIS))
  message("Unset -DOPTI_ROSVIS")
endif(OPTI_ROSVIS)

if(OPTI_ROSVIS)
  find_package(roscpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
endif()

set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

add_definitions(-DUNIT_TESTING=true)


set(SRC_FILES  ../src/quaternion.cpp
                  ../src/common.cpp
                  ../src/linalg.cpp
                  ../src/flightarea/plane.cpp
                  ../src/flightarea/flightarea.cpp
                  ../src/flightarea/flightareaconfig.cpp
                  ../src/third_party/tinyxml/XMLSerialization.cpp
                  ../src/third_party/tinyxml/tinyxml2.cpp
                  ../src/optimization/trajectory_optimization.cpp


                  main.cpp
                  quaterniontest.cpp
                  commontest.cpp
                  linalgtest.cpp
                  filtertest.cpp
                  flightareatest.cpp
                  planetest.cpp
                  trajectoryoptimizationtest.cpp
)
if(OPTI_ROSVIS)
  FILE(GLOB ROSVIS_SRCS ../src/rosvisualization/rosvisualizerdatapublisher.* ../src/rosvisualization/rosvisualizerinterface.*)
endif()


add_executable(${PROJECT_NAME} ${SRC_FILES} ${OPTI_SRC_FILES} ${ROSVIS_SRCS})

target_include_directories(${PROJECT_NAME} PRIVATE ./)
target_include_directories(${PROJECT_NAME} PRIVATE ../src)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/cam)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/flightarea)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/trackers)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/filtering)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/navigation)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/optimization)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ../src/third_party/tinyxml)

if(OPTI_ROSVIS)
  target_include_directories(${PROJECT_NAME} PRIVATE ../src/rosvisualization)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${roscpp_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${std_msgs_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${tf2_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${tf2_ros_INCLUDE_DIRS})
endif()


target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} CppUTest CppUTestExt)

if(OPTI_ROSVIS)
  target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES})
endif()
