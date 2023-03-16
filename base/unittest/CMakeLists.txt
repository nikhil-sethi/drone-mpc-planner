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

option(OPTI_ROSVIS "opti_rosviz" 0)
if(OPTI_ROSVIS)
  add_definitions(-DOPTI_ROSVIS)
  add_definitions(-DROSVIS)
  message("Set -DOPTI_ROSVIS")
else()
  remove_definitions((-DOPTI_ROSVIS))
  message("Unset -DOPTI_ROSVIS")
endif(OPTI_ROSVIS)

find_package(osqp REQUIRED)
if(OPTI_ROSVIS)
  find_package(roscpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
endif()

pkg_check_modules(CASADI REQUIRED casadi)

message("CASADI_LIBRARY_DIRS: ${CASADI_LIBRARY_DIRS}")
message("CASADI_LIBRARIES: ${CASADI_LIBRARIES}")
message("CASADI_INCLUDEDIR: ${CASADI_INCLUDEDIR}")
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")


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
                  ../src/optimization/rapid_route.cpp


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
                  rapidroutetest.cpp
)
if(OPTI_ROSVIS)
  FILE(GLOB ROSVIS_SRCS ../src/rosvisualization/rosvisualizerdatapublisher.* ../src/rosvisualization/rosvisualizerinterface.*)
endif()

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

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ~/repos/qpOASES/include)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${CASADI_INCLUDE_DIR})

if(OPTI_ROSVIS)
  target_include_directories(${PROJECT_NAME} PRIVATE ../src/rosvisualization)
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${roscpp_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${std_msgs_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${tf2_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${tf2_ros_INCLUDE_DIRS})
endif()


target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} CppUTest CppUTestExt)
target_link_libraries(${PROJECT_NAME} qpOASES)
target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARIES})
target_link_libraries(${PROJECT_NAME} osqp::osqp)
target_link_libraries(${PROJECT_NAME} osqp::osqpstatic)

if(OPTI_ROSVIS)
  target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES} ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES})
endif()

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/min_nlp/min_nlp_quadratic_optimizer.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/tti/tti_quadratic_optimizer.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/intercept_in_planes/intercept_in_planes_quadratic_optimizer.cpp DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/casadi_parser/solvertemplate_osqp.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/casadi_parser/solvertemplate_osqp.h ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/casadi_parser/solvertemplate_qpoases.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/casadi_parser/solvertemplate_qpoases.h ${CMAKE_CURRENT_SOURCE_DIR} ./../ocp_design/intercept_in_planes/ocp_intercept_in_planes.py ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/tti/ocp_tti.py
                   COMMAND make all
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../ocp_design/
)
