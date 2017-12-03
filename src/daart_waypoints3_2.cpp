#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
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
        scanVX = 0; scanVY = 0;
        forceGoalVector = 5; fovObstacleAvoidance = M_PI/5;
        minRangeScan = 0.15; maxRangeScan = 1;
        useObstacleAvoidance = true;

        ros::NodeHandle nodeLocal("~");

        std::string ns = ros::this_node::getNamespace();
        sub1 = n.subscribe("/odom", 1000, &WaypointNAV::odomCallback, this);
        sub2 = n.subscribe("/scan", 1000, &WaypointNAV::scanCallback, this);

        cmd_pub = n.advertise<geometry_msgs::Twist>(ns+"/cmd_vel", 50);
        global_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 50);
        local_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/local/goal", 50);
        oa_local_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/local/goal_oa", 50);


        robotPose.position.x = 7.5;
        robotPose.position.y = 5.5;
        robotPose.orientation = tf::createQuaternionMsgFromYaw(0);

        linearVel = nodeLocal.param("linearVel", linearVel);
        rotationVel = nodeLocal.param("rotationVel", rotationVel);
        goalAcceptedDistance = nodeLocal.param("goalAcceptedDistance", goalAcceptedDistance);
        omegaAcceptedDistance = nodeLocal.param("omegaAcceptedDistance", omegaAcceptedDistance);
        stopTime = nodeLocal.param("stopTime", stopTime);
        forceGoalVector = nodeLocal.param("forceGoalVector", forceGoalVector);
        fovObstacleAvoidance = nodeLocal.param("fovObstacleAvoidance", fovObstacleAvoidance);
        minRangeScan = nodeLocal.param("minRangeScan", minRangeScan);
        maxRangeScan = nodeLocal.param("maxRangeScan", maxRangeScan);
        useObstacleAvoidance = nodeLocal.param("useObstacleAvoidance", useObstacleAvoidance);

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
            {  17*squareScale + robotPose.position.x, -11*squareScale + robotPose.position.y},
            {  17*squareScale + robotPose.position.x,   0*squareScale + robotPose.position.y},
        };

        int idxGoal = 0;
        ros::Rate r(30);
        geometry_msgs::Twist cmd_vel;

        std::cout << "Goal sent " << std::endl;
        while(n.ok())
        {
            double angleGoal = desiredRotation(goals[idxGoal][0]-robotPose.position.x, goals[idxGoal][1]-robotPose.position.y, quaternionToYaw(robotPose.orientation));

            if (useObstacleAvoidance)
                angleGoal = obstacleAvoidanceGoal(angleGoal);

            ROS_INFO("RobotsPose: %f, %f", robotPose.position.x, robotPose.position.y);
            ROS_INFO("GoalPose: %f, %f", goals[idxGoal][0], goals[idxGoal][1]);
            ROS_INFO("Distance: %f, Bearing: %f",distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]), angleGoal);

            if (fabs(angleGoal) > omegaAcceptedDistance)
            {
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = rotationVel*sgn(angleGoal);
            }
            else if (distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]) > goalAcceptedDistance)
            {
                cmd_vel.linear.x  = linearVel;
                cmd_vel.angular.z = 2*angleGoal;
            }

            if (distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]) <= goalAcceptedDistance)
            {
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = 0.0;
                shouldStop = true;

                idxGoal++;
                if (idxGoal==4)
                    idxGoal = 0;
            }

            cmd_pub.publish(cmd_vel);
            global_goal_pub.publish( createPoseGlobalGoal(goals[idxGoal][0], goals[idxGoal][1]) );
            if (useObstacleAvoidance)
            {
                local_goal_pub.publish( createPoseLocalGoal(desiredRotation(goals[idxGoal][0]-robotPose.position.x, goals[idxGoal][1]-robotPose.position.y, quaternionToYaw(robotPose.orientation))) );
                oa_local_goal_pub.publish( createPoseLocalGoal(angleGoal) );
            }
            else
            {
                local_goal_pub.publish( createPoseLocalGoal(angleGoal) );
                oa_local_goal_pub.publish( createPoseLocalGoal( obstacleAvoidanceGoal(angleGoal) ) );
            }

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

    void scanCallback(const sensor_msgs::LaserScan scan_msg)
    {
        double scanVXT = 0, scanVYT = 0;

        int i=0;
        double angle = scan_msg.angle_min + scan_msg.angle_increment*i;
        for(; i<scan_msg.ranges.size(); i++)
        {
            if ( (fabs(angle) <= fovObstacleAvoidance*0.5)
                 && (minRangeScan < scan_msg.ranges.at(i)) && (scan_msg.ranges.at(i) < maxRangeScan)
                 && (scan_msg.ranges.at(i) != std::numeric_limits<float>::infinity())
                 && !std::isnan(scan_msg.ranges.at(i)) )
            {
                scanVXT -= (1-scan_msg.ranges.at(i))*cos(angle);
                scanVYT -= (1-scan_msg.ranges.at(i))*sin(angle);
            }
            angle += scan_msg.angle_increment;
        }
        scanVX = scanVXT; scanVY = scanVYT;
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
        double angleGoal = asin(yGoal*cos(yaw) - xGoal*sin(yaw));

        double dotGoal = xGoal*cos(yaw) + yGoal*sin(yaw);
        if (dotGoal < 0)
            angleGoal =  M_PI - angleGoal;

        return angleGoal;
    }

    double obstacleAvoidanceGoal(double angleGoal)
    {
        std::cout << "obstacle avoid scan " << std::endl;
        std::cout << angleGoal << std::endl;
        double scanVXT = scanVX + forceGoalVector*cos(angleGoal);
        double scanVYT = scanVY + forceGoalVector*sin(angleGoal);

        double d = sqrt(pow(scanVXT,2)+pow(scanVYT,2));
        scanVXT /= d;
        scanVYT /= d;
        angleGoal = atan2(scanVYT, scanVXT);
        std::cout << scanVX << std::endl;
        std::cout << scanVY << std::endl;
        std::cout << angleGoal << std::endl;
        return angleGoal;
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub1, sub2;
    ros::Publisher cmd_pub, global_goal_pub, local_goal_pub, oa_local_goal_pub;
    ros::Time last_time;
    nav_msgs::Odometry last_odom;

    bool first, shouldStop;
    double scanVX, scanVY;

    //Params
    geometry_msgs::Pose robotPose;
    double linearVel, rotationVel, goalAcceptedDistance, omegaAcceptedDistance;
    double stopTime;
    double forceGoalVector, fovObstacleAvoidance;
    double minRangeScan, maxRangeScan;
    bool useObstacleAvoidance;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "daart_waypoints3");

    WaypointNAV waypointNAV;
    waypointNAV.run();
}
