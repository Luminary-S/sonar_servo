#ifndef SONAR_SERVO_H
#define SONAR_SERVO_H

#include <mutex>
#include <thread>

#include "geometry_msgs/Quaternion.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserEcho.h"
#include <dynamic_reconfigure/server.h>
#include <sonar_servo/dynParamConfig.h>

#include "serial/serial.h"
#include "tf/tf.h"
// #include <Eigen/Core>
#include <math.h>
#include "utility.h"

// #define ENCODER_TO_ANGLE (2 * M_PI / pow(2, 12))
// #define RADIAN_TO_ENCODER (180.0 * 100 / M_PI)
// #define ERROR_THRESHOLD (5 * M_PI / 180.0)
// #define DEGREE_TO_RADIAN (M_PI / 180.0)
// #define ADJUST_MOTOR_ANGLE 0

// union, num and byte share same memory, byte high is byte[1],byte low is
// byte[0]; used for 16 to 10 transfer
union Int2Byte {
  int16_t num;
  uint8_t byte[4];
};
union char2Byte {
  unsigned short val;
  unsigned char ch[2];
};

struct EulerAngles {
  EulerAngles(const double &roll, const double &pitch, const double &yaw)
      : roll_(roll), pitch_(pitch), yaw_(yaw) {}
  EulerAngles() {}
  double roll_;
  double pitch_;
  double yaw_;
};

namespace sonar {

class SonarServo {
 public:
  SonarServo(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  // SonarServo();
  virtual ~SonarServo() {
    // if (send_motor_cmd_thread_) {
    //   delete send_motor_cmd_thread_;
    // }
  }

  // ros related
  void initParam(ros::NodeHandle &nh_private);
  void initParam();
  void define_pub_sub_server(ros::NodeHandle &nh);
  void sonarDataCallback(const sensor_msgs::LaserEchoConstPtr &msg);
  void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
  void dynParamCallback(sonar_servo::dynParamConfig &config);
  void publish();

  // controller related
  bool get_init_data(int num);
  float get_angle_interval(float des, float now);
  void sendCmd(float target, float speed, int StopBit);
  void stop();
  float speed_controller(float target, float position, float interval);
  // float position_controller(float target, int direction);
  float set_target();
  void go_to_init_pos();
  

  // motor cmd related
  void sendReceiveData(const std::vector<uint8_t> &cmd, int ret_len);
  float getEncoderData();
  void DataParser(const std::vector<uint8_t> &data);
  bool encoderDataCheck(const std::vector<uint8_t> &data);
  void packDataCmd(float Angle, float Speed, float Power, int direction,
                   int mode_num);

  //communication related
  bool initSerial();
  void getChecksum(std::vector<uint8_t> &cmd, const int start, const int end);
  bool checkCheckSum(uint8_t *cmd, const int &cmd_len);
  bool sumCheck(const std::vector<uint8_t> &cmd, const int start,
                const int end);
  bool crcCheck(unsigned char *cmd, unsigned int cmd_len);

  // struct used
  EulerAngles quaternion2Euler(const geometry_msgs::Quaternion &quat);

 private:
  std::string port_id_, motor_name_;
  double pub_rate_;
  double gain_;
  double angle_threshold_;
  int baud_rate_;
  int ranges[4];
  std::vector<float> pre_sonar;
  std::vector<float> now_sonar;
  float init_angle;
  float now_angle;
  float init_yaw;
  float now_yaw;
  float delta_yaw;
  float angle;
  float delta_init;
  int MOTOR_ID_;  // double angle[3];
  float Angles[7];
  int flag_;
  double adjust_angle_;
  float now_speed;
  int direction;
  int temp_num_;
  // float target;
  // float speed;

  // float encoder_abs_angle;
  // float pre_yaw;

  serial::Serial sonar_ser_;
  ros::Publisher servo_pub_;
  ros::Subscriber sonar_sub_;
  ros::Subscriber imu_sub_;
  dynamic_reconfigure::Server<sonar_servo::dynParamConfig> server_;
	dynamic_reconfigure::Server<sonar_servo::dynParamConfig>::CallbackType f_;

  // std::thread *send_motor_cmd_thread_;

};  // class SonarServo

}  // namespace sonar

#endif
