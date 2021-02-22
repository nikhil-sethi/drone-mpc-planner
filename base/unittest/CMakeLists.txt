project(pats-unittest)
cmake_minimum_required(VERSION 3.11.0)
find_package( OpenCV REQUIRED )


include_directories(../src)
include_directories(./)

set(SOURCE_FILES  ../src/quaternion.cpp
                  ../src/common.cpp
                  ../src/linalg.cpp
                  ../src/cameraview.cpp
                  main.cpp
                  quaterniontest.cpp
                  commontest.cpp
                  linalgtest.cpp
                  cameraviewtest.cpp)

add_executable(test_pats ${SOURCE_FILES})
target_link_libraries(test_pats ${OpenCV_LIBS} CppUTest CppUTestExt)
