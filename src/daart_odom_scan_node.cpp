#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>



class MergeOdom
{
public:

    MergeOdom ()
    {
        std::string ns = ros::this_node::getNamespace();

        sub1 = n.subscribe(ns+"/odom_encoder", 100, &MergeOdom::odomCallback, this);
        sub2 = n.subscribe(ns+"/pose2D", 100, &MergeOdom::poseCallback, this);
        odom_pub = n.advertise<nav_msgs::Odometry>(ns+"/odom", 50);
        last_time = ros::Time::now();
    }

    void poseCallback(const geometry_msgs::Pose2D pose2d_msg)
    {
        pose2d = pose2d_msg;
    }

    void odomCallback(const nav_msgs::Odometry odom_msg)
    {
        double yaw = pose2d.theta;

        double dt = (odom_msg.header.stamp - last_time).toSec();
        double dx = odom_msg.twist.twist.linear.x*dt*cos(yaw);
        double dy = odom_msg.twist.twist.linear.x*dt*sin(yaw);

        x += dx;
        y += dy;
        double omega = yaw-lastYaw;
        lastYaw = yaw;
        omega = (omega >  M_PI) ? 2*M_PI - omega : omega;
        omega = (omega < -M_PI) ? omega + 2*M_PI : omega;
        omega /= dt;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = odom_msg.header.stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "odom";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
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
        odom.child_frame_id = "odom";
        odom.twist.twist.linear.x = odom_msg.twist.twist.linear.x;
        odom.twist.twist.linear.y = odom_msg.twist.twist.linear.y;
        odom.twist.twist.angular.z = omega;

        //publish the message
        odom_pub.publish(odom);

        last_time = odom_msg.header.stamp;
    }

private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber sub1, sub2;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time last_time;
    double x=0, y=0, lastYaw=0;
    bool first = true;
    geometry_msgs::Pose2D pose2d;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "daart_odom_scan");

  MergeOdom mergeOdom;

  ros::spin();
}
