#include "sonar_servo/sonar_servo.h"

using sonar::SonarServo;

int main(int argc, char** argv) {
  ros::init(argc, argv, "sonar_servo_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  SonarServo sonar_servo(nh, nh_private);

  // ros::spin();
  return 0;
}
