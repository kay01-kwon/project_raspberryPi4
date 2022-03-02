cmake_minimum_required(VERSION 3.0.2)
project(lm4075e_pwm_control)

add_compile_options(-std=c++11)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lm4075e_pwm_control
  CATKIN_DEPENDS roscpp rospy std_msgs
  #DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${wirinPi_include}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${WIRINGPI_LIBRARIES})

add_executable(one_actuator_test src/one_lm_test.cpp include/lm4075e_pwm_control/one_lm4075e_test.cpp)
target_link_libraries(one_actuator_test ${catkin_LIBRARIES} ${WIRINGPI_LIBRARIES} -lrt)
