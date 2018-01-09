#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <deque>


using namespace std;

double quaternionToYaw(geometry_msgs::Quaternion quaternion)
{
    tf::Quaternion orientationQuaternion;
    tf::quaternionMsgToTF(quaternion, orientationQuaternion);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientationQuaternion).getRPY(roll, pitch, yaw);
    return yaw;
}

class Pose2Odom
{
public:

    Pose2Odom () : x(0), y(0), initialTh(0), child_frame_id("odom"), frame_id("map")
    {
        std::string ns = ros::this_node::getNamespace();

        sub = n.subscribe(ns+"/pose_stamped", 100, &Pose2Odom::poseCallback, this);
        odom_pub = n.advertise<nav_msgs::Odometry>(ns+"/odom", 50);

        ros::NodeHandle nodeLocal("~");
        x = nodeLocal.param("robotX", x);
        y = nodeLocal.param("robotY", y);
        z = nodeLocal.param("robotZ", z);
        initialTh = nodeLocal.param("robotTh", initialTh);

        child_frame_id = nodeLocal.param("child_frame_id", child_frame_id);
        frame_id = nodeLocal.param("frame_id", frame_id);

        initial_pose.header.stamp = ros::Time::now();

        initial_pose.pose.position.x = x;
        initial_pose.pose.position.y = y;
        initial_pose.pose.position.z = z;
        initial_pose.pose.orientation =  tf::createQuaternionMsgFromYaw(initialTh);

        last_pose.header.stamp = ros::Time::now();

        last_pose.pose.position.x = x;
        last_pose.pose.position.y = y;
        last_pose.pose.position.z = z;
        last_pose.pose.orientation =  tf::createQuaternionMsgFromYaw(initialTh);
    }

    double normalizeDAngle(double omega)
    {
        omega = (omega >  M_PI) ? (omega - 2*M_PI) : omega;
        return  (omega < -M_PI) ? (omega + 2*M_PI) : omega;
    }

    void poseCallback(geometry_msgs::PoseStamped pose_msg)
    {
        pose_msg.pose.position.x += initial_pose.pose.position.x;
        pose_msg.pose.position.y += initial_pose.pose.position.y;
        pose_msg.pose.position.z += initial_pose.pose.position.z;

        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(quaternionToYaw(pose_msg.pose.orientation) + initialTh), pose_msg.pose.orientation);

        double dt = (pose_msg.header.stamp - last_pose.header.stamp).toSec();
        double dx = (pose_msg.pose.position.x - last_pose.pose.position.x)/dt;
        double dy = (pose_msg.pose.position.y - last_pose.pose.position.y)/dt;
        double dz = (pose_msg.pose.position.z - last_pose.pose.position.z)/dt;

        double linearX = sqrt(pow(pose_msg.pose.position.x - last_pose.pose.position.x,2) + pow(pose_msg.pose.position.y - last_pose.pose.position.y,2))/dt;


        double roll, pitch, yaw;
        tf::Quaternion orientationQuaternion;
        tf::quaternionMsgToTF(pose_msg.pose.orientation, orientationQuaternion);
        tf::Matrix3x3(orientationQuaternion).getRPY(roll, pitch, yaw);

        double last_roll, last_pitch, last_yaw;
        tf::Quaternion last_orientationQuaternion;
        tf::quaternionMsgToTF(last_pose.pose.orientation, last_orientationQuaternion);
        tf::Matrix3x3(last_orientationQuaternion).getRPY(last_roll, last_pitch, last_yaw);

        double droll  = normalizeDAngle(roll  - last_roll)  / dt;
        double dpitch = normalizeDAngle(pitch - last_pitch) / dt;
        double dyaw   = normalizeDAngle(yaw   - last_yaw)   / dt;

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = pose_msg.header.stamp;
        odom_trans.header.frame_id = frame_id;
        odom_trans.child_frame_id = child_frame_id;

        odom_trans.transform.translation.x = pose_msg.pose.position.x;
        odom_trans.transform.translation.y = pose_msg.pose.position.y;
        odom_trans.transform.translation.z = pose_msg.pose.position.z;
        odom_trans.transform.rotation      = pose_msg.pose.orientation;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = pose_msg.header.stamp;
        odom.header.frame_id = frame_id;

        //set the position
        odom.pose.pose.position.x  = pose_msg.pose.position.x;
        odom.pose.pose.position.y  = pose_msg.pose.position.y;
        odom.pose.pose.position.z  = pose_msg.pose.position.z;
        odom.pose.pose.orientation = pose_msg.pose.orientation;

        //set the velocity
        odom.child_frame_id = child_frame_id;
        odom.twist.twist.linear.x = linearX;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = droll;
        odom.twist.twist.angular.y = dpitch;
        odom.twist.twist.angular.z = dyaw;

        //publish the message
        odom_pub.publish(odom);

        last_pose = pose_msg;
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    double x, y, z, initialTh;
    string child_frame_id,frame_id;

    geometry_msgs::PoseStamped last_pose, initial_pose;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "pose2odom");

  Pose2Odom pose2Odom;

  ros::spin();
}
