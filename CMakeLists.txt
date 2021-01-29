cmake_minimum_required(VERSION 3.0.2)
project(sonar_servo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
  # message_generation
  dynamic_reconfigure
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
generate_dynamic_reconfigure_options(
  config/dynParam.cfg
)


catkin_package( 
  # LIBRARIES custom_dynamic_reconfigure 
  # CATKIN_DEPENDS roscpp std_msgs dynamic_reconfigure 
) 



add_executable(sonar_servo_node 
  src/sonar_servo_node.cpp
  src/sonar_servo.cpp
  #src/crcCompute.cpp

  )


add_dependencies(sonar_servo_node ${PROJECT_NAME}_gencfg)
target_link_libraries(sonar_servo_node ${catkin_LIBRARIES})


# add_executable(sonar_servo2_node 
#   src/sonarServoNode.cpp
#   src/sonar_servo.cpp
#   #src/crcCompute.cpp

#   )
# target_link_libraries(sonar_servo2_node ${catkin_LIBRARIES})