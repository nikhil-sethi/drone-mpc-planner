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

find_package(osqp REQUIRED)
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
include_directories(../src/optimization)
include_directories(../src/third_party/tinyxml)

option(USE_OSQP "use_osqp" 0)
option(USE_QPOASES "use_qpoases" 1)

if(USE_OSQP AND USE_QPOASES)
            set(USE_OSQP 0)
            set(USE_QPOASES 1)
            message("Two optimizer libaries select, while only one is supported at the same time. Enable osqp by disabling qpoases at the same time.")
endif()

if(USE_OSQP)
  add_definitions(-DUSE_OSQP)
  remove_definitions(-DUSE_QPOASES)
  message("Set -DUSE_OSQP")
endif(USE_OSQP)

if(USE_QPOASES)
  remove_definitions(-DUSE_OSQP)
  add_definitions(-DUSE_QPOASES)
  message("Set -DUSE_QPOASES")
endif(USE_QPOASES)


add_definitions(-DOCP_DEV=true)
add_definitions(-DUNIT_TESTING=true)

set(SRC_FILES  ../src/quaternion.cpp
                  ../src/common.cpp
                  ../src/linalg.cpp
                  ../src/flightarea/plane.cpp
                  ../src/flightarea/flightarea.cpp
                  ../src/flightarea/flightareaconfig.cpp
                  ../src/optimization/sqpmethod.cpp
                  ../src/third_party/tinyxml/XMLSerialization.cpp
                  ../src/third_party/tinyxml/tinyxml2.cpp
                  ../src/optimization/intercept_in_planes_optimizer_interface.cpp
                  ../src/optimization/tti_optimizer_interface.cpp


                  main.cpp
                  ocptester.cpp
                  # minnlptest.cpp
                  ttitest.cpp
                  interceptinplanestest.cpp
                  # quaterniontest.cpp
                  # commontest.cpp
                  # linalgtest.cpp
                  # filtertest.cpp
                  # flightareatest.cpp
                  # planetest.cpp
)

if(USE_OSQP)
  set(OPTI_SRC_FILES
                  ../src/optimization/intercept_in_planes_quad_opti_osqp.cpp
                  ../src/optimization/tti_quad_opti_osqp.cpp
                  ../src/optimization/osqpconfig.cpp

      )
else()
  set(OPTI_SRC_FILES
                  ../src/optimization/intercept_in_planes_quad_opti_qpoases.cpp
                  ../src/optimization/tti_quad_opti_qpoases.cpp
                  ../src/optimization/qpoasesconfig.cpp
      )
endif()

include_directories(~/repos/qpOASES/include)
include_directories(${CASADI_INCLUDE_DIR})
link_directories(~/repos/qpOASES/build/libs)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${OPTI_SRC_FILES})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} CppUTest CppUTestExt)
target_link_libraries(${PROJECT_NAME} qpOASES)
target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARIES})
target_link_libraries(${PROJECT_NAME} osqp::osqp)
target_link_libraries(${PROJECT_NAME} osqp::osqpstatic)


add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/min_nlp/min_nlp_quadratic_optimizer.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/tti/tti_quadratic_optimizer.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/intercept_in_planes/intercept_in_planes_quadratic_optimizer.cpp DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/casadi_parser/solvertemplate_osqp.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/casadi_parser/solvertemplate_osqp.h ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/casadi_parser/solvertemplate_qpoases.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/casadi_parser/solvertemplate_qpoases.h ${CMAKE_CURRENT_SOURCE_DIR} ./../ocp_design/intercept_in_planes/ocp_intercept_in_planes.py ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/tti/ocp_tti.py
                   COMMAND make all
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/
)
