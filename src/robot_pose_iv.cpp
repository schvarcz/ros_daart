#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ibex/ibex.h>
#include <vibes/vibes.h>

using namespace ros;
using namespace ibex;

ros::Publisher odom_pub;

IntervalVector robot_pose(3,Interval(0));
bool firstImu = true;

Interval vel_bounding(-1,1),
            odom_yaw_bounding(-M_PI/36,M_PI/36),
            imu_yaw_bounding(-M_PI/36,M_PI/36);


void onNewMsg()
{
    vibes::drawBox(robot_pose,"k[y]");
}

void onNewIMU(const sensor_msgs::Imu imu)
{
    double roll=0, pitch=0, yaw=0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(imu.orientation, q);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    Interval heading = imu_yaw_bounding + yaw;
    robot_pose[2] = robot_pose[2] & heading;
    onNewMsg();
}

void onNewOdom(const nav_msgs::Odometry odom)
{
    Interval vel = vel_bounding + odom.twist.twist.linear.x;
    Interval omega = odom_yaw_bounding + odom.twist.twist.angular.z;

    double dt = (Time::now() - odom.header.stamp).toSec();
    robot_pose[0] += vel*dt*cos(robot_pose[2]);
    robot_pose[1] += vel*dt*sin(robot_pose[2]);
    robot_pose[2] += omega;
    onNewMsg();
}

int main(int argc, char **argv)
{
    // Set up ROS.
    init(argc, argv, "robot_pose_iv");
    NodeHandle n;
    Subscriber sub1 = n.subscribe("/imu", 1, onNewIMU);
    Subscriber sub2 = n.subscribe("/odom2", 1, onNewOdom);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom_iv", 50);

    vibes::beginDrawing();
    vibes::newFigure("Odometry");
    spin();
}
