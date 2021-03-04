project(pats-unittest)
cmake_minimum_required(VERSION 3.11.0)
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_STANDARD 17)

include_directories(../src)
include_directories(../src/trackers)
include_directories(../src/filtering)
include_directories(./)

set(SOURCE_FILES  ../src/quaternion.cpp
                  ../src/common.cpp
                  ../src/linalg.cpp
                  ../src/cameraview.cpp
                  main.cpp
                  quaterniontest.cpp
                  commontest.cpp
                  linalgtest.cpp
                  cameraviewtest.cpp
									filtertest.cpp
)

add_executable(test_pats ${SOURCE_FILES})
target_link_libraries(test_pats ${OpenCV_LIBS} CppUTest CppUTestExt)
