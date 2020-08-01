#include "sonar_servo/sonar_servo.h"

#include <thread>

#include "angles/angles.h"

// #include "sonar_array/crcCompute.h"

namespace sonar {

unsigned short CRC16_MODBUS(unsigned char *data, unsigned int datalen);
/*
data_len is the receive data length; cmd is the request cmd for data
*/
void SonarServo::sendReceiveData(const std::vector<uint8_t> &cmd,
                                 int data_len) {
  // send cmd
  // uint8_t GET_ENCODER_DATA_CMD[] = {0x3E, 0x90, 0x01, 0x00, 0xCF};
  int cmd_len = cmd.size();
  uint8_t CMD[cmd_len];
  for (size_t i = 0; i < cmd_len; i++) {
    CMD[i] = cmd[i] & 0xff;
  }
  // std::cout << "cmd: " << cmd[3] <<std::endl;
  sonar_ser_.write(CMD, cmd_len);

  // receive data
  // int ct = 0;
  // std::cout << "data len : " << sonar_ser_.available() << std::endl;
  // while (sonar_ser_.available() < data_len)
  // {
  //   ROS_INFO("data in buffer : %d, data len : %d", sonar_ser_.available(),
  //   data_len); uint8_t data_array[8]; sonar_ser_.read(data_array, 8);
  if (cmd_len > 6) {
    std::cout << "cmd  : ";
    std::cout.setf(std::ios::right, std::ios::adjustfield);
    std::cout.fill('0');
    for (size_t i = 0; i < cmd_len; i++) {
      std::cout << std::hex << std::setw(2) << (int)cmd[i] << "  ";
    }
    std::cout << std::dec << std::endl;
  }

  // std::cout << "in loop" << std::endl;
  //   ros::Duration(0.001).sleep();
  //   ct++;
  // }
  std::vector<uint8_t> encoder_data;
  std::vector<uint8_t> encoder_data_tmp;
  sonar_ser_.read(encoder_data_tmp, 1);
  while (true) {
    if (sonar_ser_.available() < 7) {
      ROS_WARN("data in buffer is : %d which is not enough",
               (int)sonar_ser_.available());
      // break;
    }
    // std::cout << std::hex << "first data: 0x" << (int)encoder_data_tmp[0]
    //           << "  ";
    // std::cout << std::dec << std::endl;
    // ROS_WARN("start getting 3e");
    if (0x3E == encoder_data_tmp[0]) {
      encoder_data_tmp.clear();
      sonar_ser_.read(encoder_data_tmp, 7);
      encoder_data.push_back(0x3E);

      for (size_t i = 0; i < encoder_data_tmp.size(); i++) {
        encoder_data.push_back(encoder_data_tmp[i]);
      }

      // ROS_INFO("===start checking===");
      if (!encoderDataCheck(encoder_data)) {
        ROS_WARN("receive raw data error!");
        // return cmd;/
        continue;
      }
      // ROS_INFO("end checking");
      // encoder_ser.flushInput();
      // std::cout << "encoder data : ";
      // for (size_t i = 0; i < 8; i++) {
      //   std::cout << std::hex << "0x" << (int)encoder_data[i] << "  ";
      // }
      // std::cout << std::dec << std::endl;
      // ROS_INFO("data parser");
      DataParser(encoder_data);
      // ROS_INFO("end data parser");
      break;
      //  return data;
    }
    // ct++;
  }
  // std::cout << ct * 0.001 << std::endl;
}

bool SonarServo::encoderDataCheck(const std::vector<uint8_t> &data) {
  // uint8_t d_head[5];
  // uint8_t d_data[3];
  if (data.size() == 8) {
    // for (size_t i = 0; i < 5; i++) {
    //       d_head[i] = data[i];
    //     }
    // for (size_t i = 5; i < 8; i++) {
    //   d_data[i] = data[i];
    // }
    if (sumCheck(data, 0, 4) && sumCheck(data, 5, 7)) {
      return true;
    }
  }
  return false;
}

void SonarServo::DataParser(const std::vector<uint8_t> &data) {
  // if (data.size() == 32) {
  now_angle = ((data[6] << 8) | (data[5]));  //* ENCODER_TO_ANGLE;
  // std::cout << "now bit: " << now_angle << std::endl;
  now_angle = now_angle /pow(2,12) * 2 * M_PI;//ENCODER_TO_ANGLE;
  // std::cout << "now: " << (now_angle * 1.0 * 6.28 / 2048) << std::endl;
}

float SonarServo::getEncoderData() {
  // std::cout << "start getting encoder====" << std::endl;
  // uint8_t GET_DATA_CMD[] = {0x3E, 0x90, 0x01, 0x00, 0xCF};
  // int cmd_len = sizeof(GET_DATA_CMD) / sizeof(GET_DATA_CMD[0]);
  std::vector<uint8_t> GET_DATA_CMD = {0x3E, 0x90, 0x01, 0x00, 0xCF};
  sendReceiveData(GET_DATA_CMD, 8);
  // int16_t real_position = (Data[7] << 8) | (Data[6]);
  // DataParser(Data);
  // std::cout << "end getting encoder===" << std::endl;
  return now_angle;
}

void SonarServo::stop() {
  // std::cout << "start getting encoder====" << std::endl;
  // uint8_t GET_DATA_CMD[] = {0x3E, 0x90, 0x01, 0x00, 0xCF};
  // int cmd_len = sizeof(GET_DATA_CMD) / sizeof(GET_DATA_CMD[0]);
  std::vector<uint8_t> GET_DATA_CMD = {0x3E, 0xA2, 0x01, 0x04, 0xE5,
                                       0x00, 0x00, 0x00, 0x00, 0x00};
  sendReceiveData(GET_DATA_CMD, 8);
  // int16_t real_position = (Data[7] << 8) | (Data[6]);
  // DataParser(Data);
  std::cout << "=====STOP===" << std::endl;
  // return now_angle;
}
/*
sonar array callback
*/
void SonarServo::sonarDataCallback(const sensor_msgs::LaserEchoConstPtr &msg) {
  pre_sonar = now_sonar;
  now_sonar = msg->echoes;
}
/*
imu data callback
*/
void SonarServo::imuCallback(const sensor_msgs::ImuConstPtr &msg) {
  // pre_yaw = now_yaw;
  // EulerAngles now_euler = quaternion2Euler(msg->orientation);
  // now_yaw = now_euler.yaw_;

  now_yaw = tf::getYaw(msg->orientation);
  if (flag_ == 0) {
    init_yaw = now_yaw;
    // std::cout << "callback init_yaw times: " << flag_ << "yaw= ;" << init_yaw
    //           << std::endl;
    // flag_ = 1;
  }
  flag_ += 1;
  // std::cout << "imu call back: " << tf::getYaw(msg->orientation) <<
  // std::endl;
}

/*
Mode: pos closed-loop control 3
*/
void SonarServo::packDataCmd(int64_t angle, int32_t speed, int16_t power,
                             int direction, int mode_num) {
  // uint8_t Data[];
  std::vector<uint8_t> Dat;
  std::cout << "mode_num: " << mode_num << std::endl;
  switch (mode_num) {
    case 0:  // open loop
    {
      std::cout << "get in mode 1: " << std::endl;
      int cmd_len = 8;
      uint8_t Data[cmd_len];
      Data[0] = 0x3E;
      Data[1] = 0xA0;
      Data[2] = 0X00 + MOTOR_ID_;
      Data[3] = 0x02;
      Data[4] = 0xE1;
      Data[5] = *(int8_t *)(&power);  //(unsigned char)(power & 0xff);
      Data[6] = *(int8_t *)(&power);  //(unsigned char)((power >> 8) & 0xff);
      for (size_t i = 0; i < cmd_len; i++) {
        Dat.push_back(Data[i]);
      }
      getChecksum(Dat, 0, 4);
      getChecksum(Dat, 5, 7);
      std::cout << "open loop control, only power with -850~850 is ok!"
                << std::endl;

      sendReceiveData(Dat, 8);
      break;
    }
    case 1:  //
    {
      std::cout << "get in mode 1: " << std::endl;
      int cmd_len = 10;
      uint8_t Data[cmd_len];
      Data[0] = 0x3E;
      Data[1] = 0xA2;
      Data[2] = 0X00 + MOTOR_ID_;
      Data[3] = 0x04;
      Data[4] = 0xE5;
      Data[5] = *(int8_t *)(&speed);  //(unsigned char)(vel & 0xff);
      Data[6] = *(int8_t *)(&speed);  //(unsigned char)((vel >> 8) & 0xff);
      Data[7] =
          *((int8_t *)(&speed) + 2);  //(unsigned char)((vel >> 16) & 0xff);
      Data[8] =
          *((int8_t *)(&speed) + 3);  //(unsigned char)((vel >> 24) & 0xff);
      for (size_t i = 0; i < cmd_len; i++) {
        Dat.push_back(Dat[i]);
      }
      getChecksum(Dat, 0, 4);
      getChecksum(Dat, 5, 9);
      std::cout << "vel close loop control, only vel with 32bit, 0.001dps/LSB, "
                   "36000=360dps!"
                << std::endl;

      sendReceiveData(Dat, 8);
      break;
      // int16_t real_position = (ret[7] << 8) | (ret[6]);
      // return real_position;
    }
    case 2: {
      std::cout << "get in mode 2: " << std::endl;
      int cmd_len = 14;
      uint8_t Data[cmd_len];
      Data[0] = 0x3E;
      Data[1] = 0xA3;
      Data[2] = 0X00 + MOTOR_ID_;
      Data[3] = 0x08;
      Data[4] = 0xEA;
      Data[5] = *(int8_t *)(&angle);  //(unsigned char)(angle & 0xff);
      Data[6] =
          *((int8_t *)(&angle) + 1);  //(unsigned char)((angle >> 8) & 0xff);
      Data[7] =
          *((int8_t *)(&angle) + 2);  //(unsigned char)((angle >> 16) & 0xff);
      Data[8] =
          *((int8_t *)(&angle) + 3);  //(unsigned char)((angle >> 24) & 0xff);
      Data[9] =
          *((int8_t *)(&angle) + 4);  //(unsigned char)((angle >> 32) & 0xff);
      Data[10] =
          *((int8_t *)(&angle) + 5);  //(unsigned char)((angle >> 40) & 0xff);
      Data[11] =
          *((int8_t *)(&angle) + 6);  //(unsigned char)((angle >> 48) & 0xff);
      Data[12] =
          *((int8_t *)(&angle) + 7);  //(unsigned char)((angle >> 56) & 0xff);
      for (size_t i = 0; i < cmd_len; i++) {
        Dat.push_back(Data[i]);
      }
      getChecksum(Dat, 0, 4);
      getChecksum(Dat, 5, 13);
      std::cout << "position close loop control 1, only pos with 64bit, "
                   "0.001dps/LSB, 36000=360degree, max angle is setted with "
                   "MAX_ANGLE!"
                << std::endl;

      sendReceiveData(Dat, 8);
      break;
    }
    case 3: {
      // uint8_t *Data = new uint8_t[18];
      std::cout << "get in mode 3: " << std::endl;
      int cmd_len = 18;
      uint8_t Data[cmd_len];
      Data[0] = 0x3E;
      Data[1] = 0xA4;
      Data[2] = 0X00 + MOTOR_ID_;
      Data[3] = 0x0C;
      Data[4] = 0xEF;
      Data[5] = (unsigned char)(angle & 0xff);  //*(int8_t *)(&angle);  //
      Data[6] =
          (unsigned char)((angle >> 8) & 0xff);  //*((int8_t *)(&angle) + 1); //
      Data[7] = (unsigned char)((angle >> 16) &
                                0xff);  //*((int8_t *)(&angle) + 2);  //
      Data[8] = (unsigned char)((angle >> 24) &
                                0xff);  //*((int8_t *)(&angle) + 3);  //
      Data[9] = (unsigned char)((angle >> 32) &
                                0xff);  //*((int8_t *)(&angle) + 4);  //
      Data[10] = (unsigned char)((angle >> 40) &
                                 0xff);  //*((int8_t *)(&angle) + 5);  //
      Data[11] = (unsigned char)((angle >> 48) &
                                 0xff);  //*((int8_t *)(&angle) + 6);  //
      Data[12] = (unsigned char)((angle >> 56) &
                                 0xff);  //*((int8_t *)(&angle) + 7);   //
      Data[13] = (unsigned char)(speed & 0xff);  //*(int8_t *)(&speed);  //
      Data[14] =
          (unsigned char)((speed >> 8) & 0xff);  //*((int8_t *)(&speed) + 1); //
      Data[15] = (unsigned char)((speed >> 16) &
                                 0xff);  //*((int8_t *)(&speed) + 2);  //
      Data[16] = (unsigned char)((speed >> 24) &
                                 0xff);  //*((int8_t *)(&speed) + 3);  //
      for (size_t i = 0; i < cmd_len - 1; i++) {
        Dat.push_back(Data[i]);
      }
      Dat.push_back(0);
      // std::cout << "angle finish insert... " << std::endl;
      getChecksum(Dat, 0, 4);
      getChecksum(Dat, 5, 17);
      // std::cout << "position close loop control 2, target "
      //              "position(accumulated) and max velocity !"
      //           << std::endl;

      sendReceiveData(Dat, 8);
      break;
    }
    case 4:  // using===========================
    {
      // uint8_t *Data = new uint8_t[10];
      std::cout << "get in mode 4: " << std::endl;
      int cmd_len = 10;
      uint8_t Data[cmd_len];
      Data[0] = 0x3E;
      Data[1] = 0xA5;
      Data[2] = 0X00 + MOTOR_ID_;
      Data[3] = 0x04;
      Data[4] = 0xE8;
      Data[5] = 0x00 + direction;  // 0 clockwise or 1 counter-clockwise
      Data[6] = (unsigned char)(angle & 0xff);  //*((uint8_t *)(&angle)); //
      Data[7] = (unsigned char)((angle >> 8) &
                                0xff);  //*((uint8_t *)(&angle) + 1); //
      Data[8] = (unsigned char)((angle >> 16) &
                                0xff);  //*((uint8_t *)(&angle) + 2); //

      for (size_t i = 0; i < cmd_len - 1; i++) {
        Dat.push_back(Data[i]);
      }
      Dat.push_back(0);
      // std::cout << "angle finish insert... " << std::endl;
      getChecksum(Dat, 0, 4);
      getChecksum(Dat, 5, 9);
      // std::cout << "position close loop control 3, direction and target "
      //              "position(single), max velocity == MAX_SPEED !"
      //           << std::endl;

      sendReceiveData(Dat, 8);
      break;
    }
    case 5: {
      std::cout << "get in mode 1: " << std::endl;
      int cmd_len = 14;
      uint8_t Data[cmd_len];
      Data[0] = 0x3E;
      Data[1] = 0xA6;
      Data[2] = 0X00 + MOTOR_ID_;
      Data[3] = 0x0C;
      Data[4] = 0xED;
      Data[5] = 0x00 + direction;      // 0 clockwise or 1 counter-clockwise
      Data[6] = *(uint8_t *)(&angle);  //(unsigned char)(angle & 0xff);
      Data[7] =
          *((uint8_t *)(&angle) + 1);  //(unsigned char)((angle >> 8) & 0xff);
      Data[8] =
          *((uint8_t *)(&angle) + 2);  //(unsigned char)((angle >> 16) & 0xff);
      Data[9] = *(int8_t *)(&speed);   //(unsigned char)(vel & 0xff);
      Data[10] =
          *((int8_t *)(&speed) + 1);  //(unsigned char)((vel >> 8) & 0xff);
      Data[11] =
          *((int8_t *)(&speed) + 2);  //(unsigned char)((vel >> 16) & 0xff);
      Data[12] =
          *((int8_t *)(&speed) + 3);  //(unsigned char)((vel >> 24) & 0xff);
      for (size_t i = 0; i < cmd_len; i++) {
        Dat.push_back(Data[i]);
      }
      getChecksum(Dat, 0, 4);
      getChecksum(Dat, 5, 13);

      std::cout << "position close loop control 4, direction and target "
                   "position(single), and max speed!"
                << std::endl;
      sendReceiveData(Dat, 8);
      break;
    }
    default: {
      std::cout
          << "ERROR, pls input right angle, vel, power, direction and mode_num!"
          << std::endl;
      float angle = getEncoderData();
      break;
    }
  }
}

SonarServo::SonarServo(ros::NodeHandle &nh, ros::NodeHandle &nh_private) {
  initParam(nh_private);
  ROS_INFO("init param , port id : %s", port_id_.c_str());
  if (!initSerial()) {
    ROS_WARN("Init Serial port %s failure!!", port_id_.c_str());
    return;
  }

  // servo_pub_ = nh.advertise<sensor_msgs::JointState>(motor_name_, 10);
  // use in-class function to be as subscribe callback function, should use
  // boost bind sonar_sub_ = nh.subscribe<sensor_msgs::LaserEcho>("sonar", 10,
  // boost::bind(&SonarServo::sonarDataCallback, this, _1));
  sonar_sub_ = nh.subscribe<sensor_msgs::LaserEcho>(
      "sonar", 100, &SonarServo::sonarDataCallback, this);
  imu_sub_ = nh.subscribe<sensor_msgs::Imu>("imu_data", 10,
                                            &SonarServo::imuCallback, this);
  servo_pub_ = nh.advertise<sensor_msgs::LaserEcho>(motor_name_, 10);

  if (sonar_ser_.isOpen()) {
    std::cout << "start reading serial" << std::endl;
    float temp_angle, temp_yaw;
    std::cout << "get init encoder position:" << init_angle << std::endl;
    std::cout << "start ros ok loop" << std::endl;

    // send_motor_cmd_thread_ = new std::thread(&SonarServo::loop, this);
    int num = 0;
    int temp = 0;
    int flg = 1;

    ros::Rate r(pub_rate_);
    ros::Duration(0.5).sleep();
    while (ros::ok()) {
      temp_angle = getEncoderData();
      ros::spinOnce();
      if (now_yaw > 0.0 || now_yaw < 0.0) {
        if (flg == 1) {
          temp = num;
          init_yaw = now_yaw;
        }
        flg = 0;
      } else {
        continue;
      }
      num += 1;
      ROS_INFO("============in ros loop  %d==============", num);
      // if (flg == 1) {
      if (num < 11) {
        init_angle += temp_angle;
        continue;
      } else if (num == 11) {
        init_angle = init_angle / 10.0;
        // flg2 = 0;
        // continue;
        // BOOST_TT_DETAIL_IS_MEM_FUN_POINTER_IMPL_HPP_INCLUDED
      }
      // if (num > 15) {
      //   break;
      // }
      // std::cout << "temp init_yaw: " << temp << ", init yaw = " << init_yaw
      //           << std::endl;
      // std::cout << "init_angle: " << init_angle << "; in encoder data init
      // is: "
      //           << (int)(init_angle * RADIAN_TO_ENCODER) << std::endl;

      // temp_angle = getEncoderData();
      // std::cout << "loop end" << std::endl;
      int direction = 0;
      float delta_yaw = angles::shortest_angular_distance(init_yaw, now_yaw);
      // float delta_angle =
      //     init_angle + init_yaw - now_yaw - now_angle;  // radians
      // float delta_yaw = now_yaw - init_yaw;

      // init_angle

      //  std::cout << "params========== " << std::endl;
      // nh_private.param("/sonar_servo_node/adjust_angle", adjust_angle_);
      // ROS_INFO("adjust angle: %f", adjust_angle_);
      // std::cout << "pramse======eeeeeeeeee==== " << std::endl;
      // float angle = angles::normalize_angle_positive(
      //     init_angle + delta_yaw);  // - adjust_angle_ * DEGREE_TO_RADIAN;
      float angle =
          init_angle +delta_yaw;  // - adjust_angle_ * DEGREE_TO_RADIAN;

      float delta_init = angles::shortest_angular_distance(
          angles::normalize_angle(now_angle), angles::normalize_angle(angle));
      // if (now_yaw > 0.0) {
        if (delta_yaw < 0.0) {
          direction = 1;
        } else {
          direction = 0;  // direction is depending on the yaw slope, if
                          // increasing, means it rotates counter-clockwise,
                          // servo should move clockwise, which is 0
        }
      // } else {
      //   if (delta_yaw < 0.0) {
      //     direction = 0;
      //   } else {
      //     direction = 1;  // direction is depending on the yaw slope, if
      //                     // increasing, means it rotates counter-clockwise,
      //                     // servo should move clockwise, which is 0
      //   }
      // }

      ROS_INFO(
          "init angle: %.2f, init yaw: %f; now angle: %.2f, now yaw: %f; "
          "delta_yaw: %.2f",
          angles::to_degrees(init_angle), init_yaw,
          angles::to_degrees(now_angle), now_yaw, delta_yaw);

      int data_angle =
          (int)(angles::to_degrees(angle)*100);  // angle/2/3.14 * 2048; //
      std::cout << "direction: " << direction << std::endl;
      // ROS_WARN(
      //     " next angle should be: %f, in degree %f, and in encoder data is: "
      //     "%d ",
      //     angle, (angle / DEGREE_TO_RADIAN), data_angle);
      std::string str;
      sensor_msgs::LaserEcho echo;
      Angles[0] = angles::to_degrees(now_angle);
      Angles[1] = angles::to_degrees(init_angle);
      Angles[2] = angles::to_degrees(now_yaw );// DEGREE_TO_RADIAN;
      Angles[3] = angles::to_degrees(init_yaw );//init_yaw / DEGREE_TO_RADIAN;
      Angles[4] = angles::to_degrees(delta_yaw );//;delta_yaw / DEGREE_TO_RADIAN;
      Angles[5] = angles::to_degrees(angle);
      Angles[6] = angles::to_degrees(delta_init);
      for (auto echo_data : Angles) {
        // echo_data = 0.1938 * echo_data + 53.836;
        echo.echoes.push_back(echo_data);
        // std::cout << echo_data << " ";
        str += (std::to_string(echo_data) + " ");
      }
      ROS_WARN("%s", str.c_str());
      servo_pub_.publish(echo);

      if (abs(delta_init) < angles::from_degrees(5.0)) {
        // angle = now_angle;
        ROS_INFO("== less than threshold ==");
        // continue;
        stop();
      } else {
        packDataCmd(data_angle, 0, 0, direction, 4);
      }

      if (num > 100) {
        break;
      }
      ROS_INFO("---------------end loop times: %d---------------------", num);
      r.sleep();
    }
  }
}
// void SonarReader::update_exp()
void SonarServo::loop() {}

void SonarServo::initParam(ros::NodeHandle &nh_private) {
  nh_private.param("port_id", port_id_, std::string("/dev/ttyUSB1"));
  nh_private.param("baud_rate", baud_rate_, 115200);
  nh_private.param("pub_rate", pub_rate_, 30.0);
  nh_private.param("sonar_servo_topic_name", motor_name_,
                   std::string("sonar_servo"));
  nh_private.param("adjust_angle", adjust_angle_, 0.0);
  MOTOR_ID_ = 1;
  now_yaw = 0.0;
  // pre_yaw = 0.0;
  init_angle = 0.0;
  init_yaw = 0.0;
}

bool SonarServo::initSerial() {
  try {
    sonar_ser_.setPort(port_id_);
    sonar_ser_.setBaudrate(baud_rate_);
    sonar_ser_.setParity(
        serial::parity_none);  // parity_none, parity_odd(1), ,parity_even(2)
                               // --------------
    // sonar_ser_
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    sonar_ser_.setTimeout(time_out);
    sonar_ser_.open();
  } catch (const serial::IOException &e) {
    ROS_ERROR("Unable to open port on %s -> [%s]", port_id_.c_str(), e.what());
    return false;
  }

  return true;
}

void SonarServo::getChecksum(std::vector<uint8_t> &cmd, const int start,
                             const int end) {
  int sum = 0;
  for (size_t i = start; i < end; i++) {
    sum += cmd[i];
  }
  cmd[end] = sum & 0xff;
  // cmd[end] = sum;
  // std::cout << "insert checksum: " << std::hex << "0x" << (int)cmd[end]
  // <<std::endl;
  // "
  // "; std::cout << "insert checksum: " << std::hex << "0x" << (int)sum << "
  // std::cout << std::dec << std::endl;
}

bool SonarServo::sumCheck(const std::vector<uint8_t> &cmd, const int start,
                          const int end) {
  int sum = 0;
  for (size_t i = start; i < end; i++) {
    sum += cmd[i];
  }
  int temp = sum & 0xff;

  if (temp == cmd[end]) {
    return true;
  }
  // return true;

  return false;
}

bool SonarServo::crcCheck(unsigned char *cmd, unsigned int cmd_len) {
  // unsigned char cmd[] = {0x01, 0x03, 0x00, 0x14, 0x00, 0x10, 0x04, 0x02};
  // cmd_len = 6;
  char2Byte crc;
  crc.ch[0] = cmd[cmd_len - 1];
  crc.ch[1] = cmd[cmd_len - 2];
  unsigned short temp_crc = CRC16_MODBUS(cmd, cmd_len - 2);

  // unsigned short crc = (unsigned char)(cmd[cmd_len-2]) +  (unsigned
  // char)(cmd[cmd_len-1]); std::cout << "self calculate: " <<temp_crc <<
  // std::endl; printf("%04x\n", temp_crc); printf("%04x\n", crc); std::cout
  // << "sfrom cmd: " << crc.val << std::endl;
  if (temp_crc == crc.val) return true;
  return false;
  // std::cout << "sfrom cmd: " << cmd[cmd_len-1] << std::endl;
  // return true;
}

EulerAngles SonarServo::quaternion2Euler(
    const geometry_msgs::Quaternion &quat) {
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(quat, tf_quat);

  EulerAngles euler;
  tf::Matrix3x3(tf_quat).getRPY(euler.roll_, euler.pitch_, euler.yaw_);

  return euler;
}

void InvertUint8(unsigned char *dBuf, unsigned char *srcBuf) {
  int i;
  unsigned char tmp[4] = {0};

  for (i = 0; i < 8; i++) {
    if (srcBuf[0] & (1 << i)) tmp[0] |= 1 << (7 - i);
  }
  dBuf[0] = tmp[0];
}
void InvertUint16(unsigned short *dBuf, unsigned short *srcBuf) {
  int i;
  unsigned short tmp[4] = {0};

  for (i = 0; i < 16; i++) {
    if (srcBuf[0] & (1 << i)) tmp[0] |= 1 << (15 - i);
  }
  dBuf[0] = tmp[0];
}
void InvertUint32(unsigned int *dBuf, unsigned int *srcBuf) {
  int i;
  unsigned int tmp[4] = {0};

  for (i = 0; i < 32; i++) {
    if (srcBuf[0] & (1 << i)) tmp[0] |= 1 << (31 - i);
  }
  dBuf[0] = tmp[0];
}

unsigned short CRC16_MODBUS(unsigned char *data, unsigned int datalen) {
  unsigned short wCRCin = 0xFFFF;
  unsigned short wCPoly = 0x8005;
  unsigned char wChar = 0;

  while (datalen--) {
    wChar = *(data++);
    InvertUint8(&wChar, &wChar);
    wCRCin ^= (wChar << 8);
    for (int i = 0; i < 8; i++) {
      if (wCRCin & 0x8000)
        wCRCin = (wCRCin << 1) ^ wCPoly;
      else
        wCRCin = wCRCin << 1;
    }
  }
  InvertUint16(&wCRCin, &wCRCin);
  return ((wCRCin >> 8) | (wCRCin << 8));
}

}  // namespace sonar
