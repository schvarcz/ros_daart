#include <algorithm>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

ros::Time last_time;
double omega = 0.0;
double desiredAngle = 0.0;
bool first = true, obstacleDetected = false;

using namespace std;

void odomCallback(const nav_msgs::Odometry odom)
{
    omega += odom.twist.twist.angular.z*(odom.header.stamp - last_time).toSec();
    last_time = odom.header.stamp;
}

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

double closerObstacle(sensor_msgs::LaserScan scan_msg, double centerAngle, double fov)
{
    double closestObstacle = scan_msg.range_max;
    double i = (centerAngle - fov/2.0 - scan_msg.angle_min)/scan_msg.angle_increment;
    if(i<0)
        i = 0;
    if (i > scan_msg.ranges.size())
        return scan_msg.range_min;

    for(;i < scan_msg.ranges.size() && scan_msg.angle_min + scan_msg.angle_increment*i < centerAngle+fov/2.0; i++)
        if (!isnan(scan_msg.ranges.at(i)))
            closestObstacle = min((double)scan_msg.ranges.at(i), closestObstacle);

    return closestObstacle;
}

void onNewScan(const sensor_msgs::LaserScan scan_msg)
{
    ROS_INFO("Scan: %f", closerObstacle(scan_msg, 0, M_PI/3));
    if(closerObstacle(scan_msg, 0, M_PI/3) < 0.5)
    {
//        double maxDistance = 0, maxAngle = -1;
//        for(double centerAngle=scan_msg.angle_min+M_PI/6; centerAngle+M_PI/6 <scan_msg.angle_max; centerAngle += scan_msg.angle_increment)
//        {
//            double distObstacle = closerObstacle(scan_msg, 0, M_PI/3);
//            if(distObstacle > maxDistance)
//            {
//                maxDistance = distObstacle;
//                maxAngle = centerAngle;
//            }
//        }
//        desiredAngle = maxAngle;

//        for( int i=1; M_PI/6 +scan_msg.angle_increment*i <scan_msg.angle_max; i++)
//        {
//            double distObstacle = closerObstacle(scan_msg, scan_msg.angle_increment*i, M_PI/3);
//            if(distObstacle > 0.5)
//            {
//                desiredAngle = scan_msg.angle_increment*i;
//            }

//            distObstacle = closerObstacle(scan_msg, -scan_msg.angle_increment*i, M_PI/3);
//            if(distObstacle > 0.5)
//            {
//                desiredAngle = -scan_msg.angle_increment*i;
//            }
//        }

        double maxDistance = 0, maxAngle = -1;
        for( int i=1; M_PI/6 +scan_msg.angle_increment*i <scan_msg.angle_max; i++)
        {
            double distObstacle = closerObstacle(scan_msg, scan_msg.angle_increment*i, M_PI/3);
            if(distObstacle > maxDistance)
            {
                maxDistance = distObstacle;
                maxAngle = scan_msg.angle_increment*i;
            }

            distObstacle = closerObstacle(scan_msg, -scan_msg.angle_increment*i, M_PI/3);
            if(distObstacle > maxDistance)
            {
                maxDistance = distObstacle;
                maxAngle = -scan_msg.angle_increment*i;
            }
        }
        desiredAngle = maxAngle;


        omega = 0;
        obstacleDetected = true;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "daart_random_walk");

  ros::NodeHandle n;
  std::string ns = "/robot0"; //ros::this_node::getNamespace();
  ros::Subscriber sub1 = n.subscribe(ns+"/odom", 100, odomCallback);
  ros::Subscriber sub2 = n.subscribe(ns+"/laser_0", 100, onNewScan);
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>(ns+"/cmd_vel", 50);

  ros::Rate mRate(30);
  while(n.ok())
  {
    geometry_msgs::Twist cmd_vel;

    if(obstacleDetected && fabs(omega-desiredAngle) <= M_PI/36)
        obstacleDetected = false;

    if (obstacleDetected)
    {
        cmd_vel.angular.z = 1*sgn(desiredAngle-omega);
        cmd_pub.publish(cmd_vel);
        ROS_INFO("z = %f",cmd_vel.angular.z);
    }
    else if (!obstacleDetected)
    {
        ROS_INFO("Forward.");
        cmd_vel.linear.x = 1;
        cmd_vel.angular.z = 0.;
        cmd_pub.publish(cmd_vel);
        ROS_INFO("x = %f",cmd_vel.linear.x);
    }

    ros::spinOnce();
    mRate.sleep();
  }
}
