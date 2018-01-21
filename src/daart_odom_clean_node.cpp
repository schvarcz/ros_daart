#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <deque>
#include <tf/transform_broadcaster.h>


using namespace std;
class OdomClean
{
public:

    OdomClean () : topic2clean("/odom_encoder"), topicPub("/odom"), childFrame("base_link"), frameId("odom")
    {
        ros::NodeHandle nodeLocal("~");

        topic2clean = nodeLocal.param("topic2clean", topic2clean);
        topicPub = nodeLocal.param("topicPub", topicPub);

        childFrame = nodeLocal.param("childFrame", childFrame);
        frameId = nodeLocal.param("frameId", frameId);

        std::string ns = ros::this_node::getNamespace();

        sub1 = n.subscribe(ns+topic2clean, 100, &OdomClean::odomCallback, this);
        odom_pub = n.advertise<nav_msgs::Odometry>(ns+topicPub, 50);
    }

    void odomCallback(const nav_msgs::Odometry odom_msg)
    {
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = odom_msg.header.stamp;
        odom_trans.header.frame_id = frameId;
        odom_trans.child_frame_id = childFrame;

        odom_trans.transform.translation.x = odom_msg.pose.pose.position.x;
        odom_trans.transform.translation.y = odom_msg.pose.pose.position.y;
        odom_trans.transform.translation.z = odom_msg.pose.pose.position.z;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom = odom_msg;
        odom.header.frame_id = frameId;
        odom.child_frame_id = childFrame;

        //set the orientation
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.twist.twist.angular.z = 0;

        //publish the message
        odom_pub.publish(odom);
    }

private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber sub1;
    tf::TransformBroadcaster odom_broadcaster;
    string topic2clean, topicPub, childFrame, frameId;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "daart_odom_clean");

  OdomClean odomClean;

  ros::spin();
}
