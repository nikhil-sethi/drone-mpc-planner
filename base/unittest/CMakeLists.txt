project(pats-unittest)
cmake_minimum_required(VERSION 3.11.0)
find_package( OpenCV REQUIRED )
find_package(PkgConfig REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++fs -Wall -Wextra -Wlogical-op -Wuseless-cast -Wold-style-cast -Wdouble-promotion -Wshadow -Wformat=2  -Wno-missing-field-initializers")
MESSAGE( STATUS "CMAKE BUILD TYPE: " ${CMAKE_BUILD_TYPE} )

pkg_check_modules(CASADI REQUIRED casadi)

message("CASADI_LIBRARY_DIRS: ${CASADI_LIBRARY_DIRS}")
message("CASADI_LIBRARIES: ${CASADI_LIBRARIES}")
message("CASADI_INCLUDEDIR: ${CASADI_INCLUDEDIR}")
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

include_directories(./)
include_directories(../src)
include_directories(../src/cam)
include_directories(../src/flightarea)
include_directories(../src/trackers)
include_directories(../src/filtering)
include_directories(../src/navigation)
include_directories(../ocp_design/intercept_in_planes)
include_directories(../ocp_design/tti)
include_directories(../ocp_design/min_nlp)

add_definitions(-DOCP_DEV=true)

add_definitions(-DUNIT_TESTING)

set(SOURCE_FILES  ../src/quaternion.cpp
                  ../src/common.cpp
                  ../src/linalg.cpp
                  ../src/flightarea/plane.cpp
                  ../src/flightarea/flightarea.cpp
                  ../src/flightarea/flightareaconfig.cpp
                  ../ocp_design/intercept_in_planes/intercept_in_planes_optimizer_interface.cpp
                  ../ocp_design/intercept_in_planes/intercept_in_planes_quadratic_optimizer.cpp
                  ../ocp_design/tti/tti_optimizer_interface.cpp
                  ../ocp_design/tti/tti_quadratic_optimizer.cpp
                  # ../ocp_design/min_nlp/min_nlp_optimizer_interface.cpp
                  # ../ocp_design/min_nlp/min_nlp_quadratic_optimizer.cpp
                  ../src/sqpmethod.cpp

                  main.cpp
                  ocptester.cpp
                  # minnlptest.cpp
                  ttitest.cpp
                  interceptinplanestest.cpp
                  # quaterniontest.cpp
                  # commontest.cpp
                  # linalgtest.cpp
                  # filtertest.cpp
                  flightareatest.cpp
                  # planetest.cpp
)

include_directories(~/repos/qpOASES/include)
include_directories(${CASADI_INCLUDE_DIR})
link_directories(~/repos/qpOASES/build/libs)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} CppUTest CppUTestExt)
target_link_libraries(${PROJECT_NAME} qpOASES)
target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARIES})
