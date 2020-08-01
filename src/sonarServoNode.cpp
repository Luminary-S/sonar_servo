/*******************************************************
 * Copyright (C) 2020, Chinese University of Hong Kong, T Stone Robotics Institute 
 * 
 * This file is part of Sonar Servo.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: SUN Guangli (cuhksgl@gmail.com)
 *******************************************************/
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <ros/console.h>

#include "sonar_servo/sonar_servo.h"

using sonar::SonarServo;
SonarServo sonarServo;

using namespace std;
// using namespace ros;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::LaserEchoConstPtr> laser_buf;
// queue<sensor_msgs::PointCloudConstPtr> feature_buf;
// queue<sensor_msgs::ImageConstPtr> img0_buf;
// queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

// 

void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg){

}

void sonarDataCallback(const sensor_msgs::LaserEchoConstPtr &msg){

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar estimator");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    // readParameters(config_file);
    // estimator.setParameter();

// #ifdef EIGEN_DONT_PARALLELIZE
//     ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
// #endif

    ROS_WARN("waiting for image and imu...");

    // registerPub(n);

    ros::Subscriber sub_imu = nh.subscribe("/imu_data", 1000, imuCallback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_feature = nh.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    // ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    // ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = nh.subscribe("/sonar", 100, sonarDataCallback);


    // std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
