#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ibex/ibex.h>
#include <vibes/vibes.h>
#include <iostream>

using namespace ros;
using namespace std;
using namespace ibex;

ros::Publisher odom_pub;

IntervalVector robot_pose(3,Interval(-0.1,0.1));
bool firstImu = true;

Interval vel_bounding(0),
            odom_yaw_bounding(0),
            imu_yaw_bounding(0);


void onNewMsg()
{
    vibes::drawBox(robot_pose,"k[y]");
    vibes::drawAUV(0,  50, robot_pose[2].mid()*180./M_PI, 1, "k[y]");
}

void onNewIMU(const sensor_msgs::Imu imu)
{
    double roll=0, pitch=0, yaw=0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(imu.orientation, q);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    Interval heading = imu_yaw_bounding + yaw;
    robot_pose[2] = heading;
    cout << robot_pose[2] << heading << yaw << endl;
    onNewMsg();
}

void onNewOdom(const nav_msgs::Odometry odom)
{
    Interval vel = vel_bounding + odom.twist.twist.linear.x*2.4;
    Interval omega = odom_yaw_bounding + odom.twist.twist.angular.z/2.5;

    double dt = (Time::now() - odom.header.stamp).toSec();

    ROS_INFO("vel: %f, omega: %f, dt: %f... ", odom.twist.twist.linear.x*2.4, odom.twist.twist.angular.z/2.5, dt);
    robot_pose[0] += vel*cos(robot_pose[2]);
    robot_pose[1] += vel*sin(robot_pose[2]);
    robot_pose[2] += omega;
    ROS_INFO("X: %f, Y: %f, Yaw: %f... ", robot_pose[0].mid(), robot_pose[1].mid(), robot_pose[2].mid());
    onNewMsg();
}

int main(int argc, char **argv)
{
    robot_pose[2] = Interval(M_PI);
    // Set up ROS.
    init(argc, argv, "robot_pose_iv");
    NodeHandle n;
//    Subscriber sub1 = n.subscribe("/imu", 1, onNewIMU);
    Subscriber sub2 = n.subscribe("/odom", 1, onNewOdom);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom_iv", 50);

    vibes::beginDrawing();
    vibes::newFigure("Odometry");
    spin();
}
