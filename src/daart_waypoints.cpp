#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

ros::Time last_time;
geometry_msgs::Pose robotPose;
double omega = 0.0;
bool first = true, odomReceived = false;

void odomCallback(const nav_msgs::Odometry odom)
{
    double dt = (odom.header.stamp - last_time).toSec();
    omega += odom.twist.twist.angular.z*dt;
    robotPose = odom.pose.pose;
    last_time = odom.header.stamp;
    odomReceived = true;
}

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

double distance(double x0, double y0, double x1, double y1)
{
    return sqrt(pow(x1-x0,2) + pow(y1-y0,2));
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "daart_waypoints");

    ros::NodeHandle n;
    std::string ns = ros::this_node::getNamespace();
    ros::Subscriber sub = n.subscribe(ns+"/odom", 100, odomCallback);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>(ns+"/cmd_vel", 50);
    bool sended = false;

//  double goals[][2] = {
//      { 1 + 6.5, 0 + 8.5},
//      {-1 + 6.5, 0 + 8.5},
//      { 0 + 6.5,-1 + 8.5},
//  };
// double goals[][2] = {
//     {  7.5, 6.9},
//     {  7.5, 5.2},
//     { 12.4, 5  },
//     { 12.4, 2.5},
//     { 16  , 2.5},
//     { 16  , 5  },
// };

    double squareScale = 0.31;
    double goals[][2] = {
        {  30*squareScale + 7, 0 + 5},
        {  30*squareScale + 7, -11*squareScale + 5},
        {  15*squareScale + 7, -11*squareScale + 5},
        {  15*squareScale + 7, 0 + 5},
    };

    int idxGoal = 0;
    robotPose.position.x = 7;
    robotPose.position.y = 5;
    double roll, pitch, yaw = 0;
    ros::Rate r(30);
    while(n.ok())
    {
        ROS_INFO("RobotsPose: %f, %f", robotPose.position.x, robotPose.position.y);
        ROS_INFO("GoalPose: %f, %f", goals[idxGoal][0], goals[idxGoal][1]);

        double angleGoal = desiredRotation(goals[idxGoal][0] - robotPose.position.x, goals[idxGoal][1] - robotPose.position.y, yaw);

        double diffAngle = angleGoal-omega;
        geometry_msgs::Twist cmd_vel;

        if(fabs(diffAngle) > M_PI/90)
        {
            ROS_INFO("angleGoal: %f",angleGoal);
            ROS_INFO("diffAngle: %f",diffAngle);

            cmd_vel.linear.x = 0.;
            cmd_vel.angular.z = 0.5*sgn(diffAngle);
            sended = true;
            cmd_pub.publish(cmd_vel);
            if (first)
            {
                last_time = ros::Time::now();
                first = false;
            }
            ROS_INFO("z = %f",cmd_vel.angular.z);
        }
        else if (distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]) > 0.3)
        {
            ROS_INFO("Distance: %f",distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]));
            cmd_vel.linear.x = 0.5;
            cmd_vel.angular.z = 0.;
            sended = true;
            cmd_pub.publish(cmd_vel);
            if (first)
            {
                last_time = ros::Time::now();
                first = false;
            }
            ROS_INFO("x = %f",cmd_vel.linear.x);
        }
        else
        {
            omega = 0.0;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0;
            sended = false;
            cmd_pub.publish(cmd_vel);
            ROS_INFO("z = %f",cmd_vel.angular.z);

            tf::Quaternion q;
            tf::quaternionMsgToTF(robotPose.orientation, q);
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);

            idxGoal++;
            if (idxGoal==4)
                idxGoal = 0;
        }


        ros::spinOnce();
        if (!sended)
            sleep(5);
        r.sleep();
    }
}
