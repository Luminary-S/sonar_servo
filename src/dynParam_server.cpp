#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include </sonar_servo/gain_Config.h>

void callback(sonar_servo::gain_Config &config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: %f",
		// config.int_param,
		config.speed_gain
		// config.str_param.c_str(),
		// config.bool_param?"True":"False",
		// config.size
    );
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sonar_servo_node_dynamic_reconfigure");
	dynamic_reconfigure::Server<test::Tutorials> server_;
	dynamic_reconfigure::Server<test::Tutorials>::CallbackType f_;
	f = boost::bind(&callback, _1, _2);//绑定回调函数
	//为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况
    server_.setCallback(f_);
	ros::spin();
	return 0;

}