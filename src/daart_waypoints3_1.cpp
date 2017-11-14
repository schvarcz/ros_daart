#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

double distance(double x0, double y0, double x1, double y1)
{
    return sqrt(pow(x1-x0,2) + pow(y1-y0,2));
}

double quaternionToYaw(geometry_msgs::Quaternion quaternion)
{
    tf::Quaternion orientationQuaternion;
    tf::quaternionMsgToTF(quaternion, orientationQuaternion);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientationQuaternion).getRPY(roll, pitch, yaw);
    return yaw;
}

class WaypointNAV
{

public:
    WaypointNAV() : first(true), shouldStop(false)
    {
        linearVel = 0.2; rotationVel = 1.4; goalAcceptedDistance = 0.05; omegaAcceptedDistance = M_PI/90;
        stopTime = 5000000;

        ros::NodeHandle nodeLocal("~");

        std::string ns = ros::this_node::getNamespace();
        sub = n.subscribe(ns+"/odom", 100, &WaypointNAV::odomCallback, this);

        cmd_pub = n.advertise<geometry_msgs::Twist>(ns+"/cmd_vel", 50);
        global_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 50);
        local_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/local/goal", 50);


        robotPose.position.x = 7.5;
        robotPose.position.y = 5.5;
        robotPose.orientation = tf::createQuaternionMsgFromYaw(0);

        linearVel = nodeLocal.param("linearVel", linearVel);
        rotationVel = nodeLocal.param("rotationVel", rotationVel);
        goalAcceptedDistance = nodeLocal.param("goalAcceptedDistance", goalAcceptedDistance);
        omegaAcceptedDistance = nodeLocal.param("omegaAcceptedDistance", omegaAcceptedDistance);
        stopTime = nodeLocal.param("stopTime", stopTime);

        robotPose.position.x = nodeLocal.param("robotX", robotPose.position.x);
        robotPose.position.y = nodeLocal.param("robotY", robotPose.position.y);
        robotPose.orientation = tf::createQuaternionMsgFromYaw( nodeLocal.param("robotTh", quaternionToYaw(robotPose.orientation)) );
    }

    void run()
    {
        double squareScale = 0.31;
        double goals[][2] = {
            {  30*squareScale + robotPose.position.x,   0*squareScale + robotPose.position.y},
            {  30*squareScale + robotPose.position.x, -11*squareScale + robotPose.position.y},
            {  15*squareScale + robotPose.position.x, -11*squareScale + robotPose.position.y},
            {  15*squareScale + robotPose.position.x,   0*squareScale + robotPose.position.y},
        };

        int idxGoal = 0;
        ros::Rate r(30);
        geometry_msgs::Twist cmd_vel;

        std::cout << "Goal sent " << std::endl;
        while(n.ok())
        {
            double angleGoal = desiredRotation(goals[idxGoal][0]-robotPose.position.x, goals[idxGoal][1]-robotPose.position.y, quaternionToYaw(robotPose.orientation));

            ROS_INFO("RobotsPose: %f, %f", robotPose.position.x, robotPose.position.y);
            ROS_INFO("GoalPose: %f, %f", goals[idxGoal][0], goals[idxGoal][1]);
            ROS_INFO("Distance: %f, Bearing: %f",distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]), angleGoal);

            if(fabs(angleGoal) > omegaAcceptedDistance)
            {
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = rotationVel*sgn(angleGoal);
            }
            else if (distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]) > goalAcceptedDistance)
            {
                cmd_vel.linear.x  = linearVel;
                cmd_vel.angular.z = rotationVel*sin(angleGoal);
            }

            if (distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]) < goalAcceptedDistance)
            {
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = 0.0;
                shouldStop = true;

                idxGoal++;
                if (idxGoal==4)
                    idxGoal = 0;
            }

            cmd_pub.publish(cmd_vel);
            local_goal_pub.publish( createPoseLocalGoal(angleGoal) );
            global_goal_pub.publish( createPoseGlobalGoal(goals[idxGoal][0], goals[idxGoal][1]) );

            ROS_INFO("vel = %f\tomega = %f\n",cmd_vel.linear.x, cmd_vel.angular.z);

            if (first)
                last_time = ros::Time::now();

            if (shouldStop)
            {
                shouldStop = false;
                usleep(stopTime);
            }

            ros::spinOnce();
            r.sleep();
        }
    }

    void odomCallback(const nav_msgs::Odometry odom)
    {
        double dt = (odom.header.stamp - last_time).toSec();

        robotPose = odom.pose.pose;
        last_time = odom.header.stamp;
        last_odom = odom;
        if (first)
            first = false;
    }

    geometry_msgs::PoseStamped createPoseGlobalGoal(double x, double y)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        return pose;
    }

    geometry_msgs::PoseStamped createPoseLocalGoal(double angle)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = last_odom.pose.pose.position.x;
        pose.pose.position.y = last_odom.pose.pose.position.y;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        return pose;
    }

    double desiredRotation(double xGoal, double yGoal, double yaw)
    {
        double mA = distance(0, 0, xGoal, yGoal);
        xGoal /= mA; yGoal /= mA;
        double dotGoal = xGoal*cos(yaw) + yGoal*sin(yaw);
        double angleGoal = asin(yGoal*cos(yaw) - xGoal*sin(yaw));
        if (dotGoal < 0)
            angleGoal =  sgn(angleGoal)*M_PI - angleGoal;
        return angleGoal;
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher cmd_pub, global_goal_pub, local_goal_pub;
    ros::Time last_time;
    geometry_msgs::Pose robotPose;
    nav_msgs::Odometry last_odom;
    bool first, shouldStop;
    double linearVel, rotationVel, goalAcceptedDistance, omegaAcceptedDistance;
    double stopTime;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "daart_waypoints3");

    WaypointNAV waypointNAV;
    waypointNAV.run();
}
