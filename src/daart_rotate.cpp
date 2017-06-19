//#include <stdio.h>
//#include <stdlib.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <linux/i2c-dev.h>
//#include <string.h>
//#include <stdint.h>
//#include <unistd.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>



ros::Time last_time;
double omega = 0.0;
bool first = true;

void odomCallback(const nav_msgs::Odometry odom)
{
    omega += odom.twist.twist.angular.z*(odom.header.stamp - last_time).toSec();
    last_time = odom.header.stamp;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "daart_rotate");

  ros::NodeHandle n;
  std::string ns = "/robot0";// ros::this_node::getNamespace();
  ros::Subscriber sub = n.subscribe(ns+"/odom", 100, odomCallback);
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>(ns+"/cmd_vel", 50);
  bool sended = false;

  ros::Rate r(30);
  while(n.ok())
  {

    geometry_msgs::Twist cmd_vel;
    if(omega >= M_PI_2)
    {
        omega = 0.0;
        cmd_vel.angular.z = 0;
        sended = false;
        cmd_pub.publish(cmd_vel);
        ROS_INFO("z = %f",cmd_vel.angular.z);
    }
    else
    {
        cmd_vel.angular.z = 1;
        sended = true;
        cmd_pub.publish(cmd_vel);
        ROS_INFO("z = %f",cmd_vel.angular.z);
    }


    ros::spinOnce();
    if (!sended)
        sleep(5);
    r.sleep();
  }
}
