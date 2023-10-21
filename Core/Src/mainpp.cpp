#include "mainpp.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
//#include "std_msgs/MultiArrayLayout.h"
//#include "std_msgs/MultiArrayDimension.h"

void vel_callback(const geometry_msgs::Twist &msg)
{

   vel[0] = msg.linear.x;  //double vel[3]宣告在mainpp.h
   vel[1] = msg.linear.y;
   vel[2] = msg.angular.z;


}

void point_callback(const geometry_msgs::Point &msg_1)
{
	x = msg_1.x;
	y = msg_1.y;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("wheel_vel", vel_callback);
ros::Subscriber<geometry_msgs::Point> sub1("Destination", point_callback);
std_msgs::Float64MultiArray msg;
ros::Publisher encoder_pub("encoder_data",&msg);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->flush();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    nh.getHardware()->reset_rbuf();
}
void setup(void)
{
    nh.initNode();
    nh.subscribe(sub);
    nh.subscribe(sub1);
    nh.advertise(encoder_pub);
}
void loop(void)
{
	msg.data = encoder;
	msg.data_length = 4;
	encoder_pub.publish(&msg);
    nh.spinOnce();
}
