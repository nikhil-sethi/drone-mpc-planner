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

add_executable(${PROJECT_NAME} ${SRC_FILES} ${OPTI_SRC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ./)
target_include_directories(${PROJECT_NAME} PRIVATE ../src)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/cam)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/flightarea)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/trackers)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/filtering)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/navigation)
target_include_directories(${PROJECT_NAME} PRIVATE ../src/optimization)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ../src/third_party/tinyxml)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} CppUTest CppUTestExt)
