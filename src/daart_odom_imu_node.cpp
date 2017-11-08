#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

using namespace ros;

ros::Publisher scan_pub;
ros::Time last_time;
bool firstMsg = true;
double x=0, y=0, lastYaw;

double roll=0, pitch=0, yaw=0;

void onNewIMU(const sensor_msgs::Imu imu)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(imu.orientation, q);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

void onNewOdom(const nav_msgs::Odometry odom_msg)
{
    double dt;
    dt = (odom_msg.header.stamp - last_time).toSec();

    double dx = odom_msg.twist.twist.linear.x*dt*cos(yaw);
    double dy = odom_msg.twist.twist.linear.x*dt*sin(yaw);

    x += dx;
    y += dy;
    double omega = fabs(yaw-lastYaw);
    omega = omega > M_PI ? 2*M_PI - omega : omega;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = odom_msg.header.stamp;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "odom_imu";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tf::TransformBroadcaster odom_broadcaster;
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = odom_msg.header.stamp;
    odom.header.frame_id = "map";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "odom_imu";
    odom.twist.twist.linear.x = odom_msg.twist.twist.linear.x;
    odom.twist.twist.linear.y = odom_msg.twist.twist.linear.y;
    odom.twist.twist.angular.z = omega;

    //publish the message
    scan_pub.publish(odom);

    last_time = odom_msg.header.stamp;
}

int main(int argc, char **argv)
{
    // Set up ROS.
    init(argc, argv, "daart_odom_imu_node");
    NodeHandle n;
    Subscriber sub1 = n.subscribe("/imu", 1, onNewIMU);
    Subscriber sub2 = n.subscribe("/odom", 1, onNewOdom);
    scan_pub = n.advertise<nav_msgs::Odometry>("odom_imu", 50);

    spin();
}
