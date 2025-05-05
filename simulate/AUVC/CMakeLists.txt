cmake_minimum_required(VERSION 3.16)

# add_subdirectory(apriltags)

# TODO : Test this
find_package(OpenCV)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(RoverPhysics SHARED roverPhysics.h roverPhysics.cc)
set_property(TARGET RoverPhysics PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(RoverPhysics PROPERTIES COMPILE_FLAGS "-fvisibility=default")
target_link_libraries(RoverPhysics PUBLIC mujoco::mujoco) # TODO:Test and remove this

add_library(Controller SHARED controller.h controller.cc)
set_property(TARGET Controller PROPERTY POSITION_INDEPENDENT_CODE ON)
set_target_properties(Controller PROPERTIES COMPILE_FLAGS "-fvisibility=default")
target_link_libraries(Controller PUBLIC mujoco::mujoco) # TODO:Test and remove this




find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB APRILTAG_HEADERS AR_SRC/*.h)
file(GLOB APRILTAG_SRCS AR_SRC/*.cc)

add_library(CustomTag STATIC ${APRILTAG_HEADERS}
                             ${APRILTAG_SRCS}
                             Tags/Serial.h
                             Tags/Serial.cc
                             Tags/tags.h
                             Tags/tags.cc)

target_include_directories(CustomTag PUBLIC ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
list(APPEND CMAKE_INCLUDE_PATH "/usr/include")
target_link_libraries(CustomTag PRIVATE mujoco::mujoco Eigen3::Eigen ${OpenCV_LIBS})

# if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#   target_link_libraries(CustomTag -L/opt/local/lib/) # MacPorts special treatment...
# else (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#   target_link_libraries(CustomTag PUBLIC v4l2 )
# endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
