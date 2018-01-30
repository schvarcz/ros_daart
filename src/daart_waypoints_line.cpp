#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <math.h>

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
    WaypointNAV() : first(true), shouldStop(false), maxDist(1.0)
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
        path_goal_pub = n.advertise<visualization_msgs::Marker>("/move_base_simple/goal_path", 0);


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

        maxDist = nodeLocal.param("maxDist", maxDist);



        baseLine.id = 0;
        baseLine.scale.x = 0.2;
        baseLine.header.frame_id = "map"; // ??
        baseLine.action = visualization_msgs::Marker::MODIFY;
        baseLine.type = visualization_msgs::Marker::LINE_STRIP;
        baseLine.color.a = 1.0; baseLine.color.r = 0.0; baseLine.color.g = 0.0; baseLine.color.b = 1.0;

        goalLine.id = 1;
        goalLine.scale.x = 0.1;
        goalLine.header.frame_id = "map"; // ??
        goalLine.action = visualization_msgs::Marker::MODIFY;
        goalLine.type = visualization_msgs::Marker::LINE_STRIP;
        goalLine.color.a = 1.0; goalLine.color.r = 1.0; goalLine.color.g = 0.0; goalLine.color.b = 0.0;
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

        for(int i =0; i<3;i++)
        {
          sleep(3);
          ros::spinOnce();
        }
        createGoal(goals[idxGoal][0], goals[idxGoal][1]);
        path_goal_pub.publish(baseLine);
        path_goal_pub.publish(goalLine);
        std::cout << "Goal sent " << std::endl;

        while(n.ok())
        {
            double angleGoal = desiredRotation(quaternionToYaw(robotPose.orientation));

            if (useObstacleAvoidance)
            {
                ROS_INFO("Using obstacle avoidance.");
                angleGoal = obstacleAvoidanceGoal(angleGoal);
            }

            ROS_INFO("RobotsPose: %f, %f", robotPose.position.x, robotPose.position.y);
            ROS_INFO("GoalPose: %f, %f", goals[idxGoal][0], goals[idxGoal][1]);
            ROS_INFO("Distance: %f, Bearing: %f",distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]), angleGoal);

            if ( (fabs(angleGoal) > 2*omegaAcceptedDistance)
                || ((fabs(angleGoal) > omegaAcceptedDistance) && (distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]) > 2*goalAcceptedDistance)) )
            {
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = rotationVel*sgn(angleGoal);
            }
            else if (distance(robotPose.position.x, robotPose.position.y, goals[idxGoal][0], goals[idxGoal][1]) > goalAcceptedDistance)
            {
                cmd_vel.linear.x  = linearVel;
                cmd_vel.angular.z = 2*angleGoal;
            }

            if (goalReached())
            {
                cmd_vel.linear.x  = 0.0;
                cmd_vel.angular.z = 0.0;
                shouldStop = true;

                idxGoal++; //Change the goal
                if (idxGoal==4)
                    idxGoal = 0;

                createGoal(goals[idxGoal][0], goals[idxGoal][1]);
            }
            path_goal_pub.publish(baseLine);
            path_goal_pub.publish(goalLine);

            cmd_pub.publish(cmd_vel);
            global_goal_pub.publish( createPoseGlobalGoal(goals[idxGoal][0], goals[idxGoal][1]) );

            if (useObstacleAvoidance)
            {
                local_goal_pub.publish( createPoseLocalGoal(desiredRotation(quaternionToYaw(robotPose.orientation))) );
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
            angleGoal =  sgn(angleGoal)*M_PI - angleGoal;

        return angleGoal;
    }

    double desiredRotation(double yaw)
    {
        double desiredAngle = getDesiredAngle();
        double angleGoal = asin(sin(desiredAngle)*cos(yaw) - cos(desiredAngle)*sin(yaw));

        double dotGoal = cos(desiredAngle)*cos(yaw) + sin(desiredAngle)*sin(yaw);
        if (dotGoal < 0)
            angleGoal =  sgn(angleGoal)*M_PI - angleGoal;

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

    bool goalReached()
    {
        geometry_msgs::Point pt1 = goalLine.points[0];
        geometry_msgs::Point pt2 = goalLine.points[1];
        double x = robotPose.position.x, y = robotPose.position.y, x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y; //Goal line
        return ((x-x1)*(y2-y1) - (y-y1)*(x2-x1)) > 0; //When it becomes positive, we passed the line
    }

    double getAngle()
    {
        geometry_msgs::Point pt1 = baseLine.points[0];
        geometry_msgs::Point pt2 = baseLine.points[1];
        double x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y;
        return atan2(y2-y1, x2-x1); //When it becomes positive, we passed the line
    }

    double getDesiredAngle()
    {
        return getAngle() + M_PI_2*std::max(std::min(distRobot2Line()/maxDist, 1.0),-1.0);
    }

    double distRobot2Line()
    {
        geometry_msgs::Point pt1 = baseLine.points[0];
        geometry_msgs::Point pt2 = baseLine.points[1];
        double x = robotPose.position.x, y = robotPose.position.y, x1 = pt1.x, y1 = pt1.y, x2 = pt2.x, y2 = pt2.y;
        return ((y2-y1)*x - (x2-x1)*y + x2*y1 - y2*x1) / sqrt(pow(y2-y1,2) + pow(x2-x1,2));
    }

    geometry_msgs::Point rotatePt(geometry_msgs::Point pt, double angle)
    {
        double x, y;
        x = pt.x*cos(angle) - pt.y*sin(angle);
        y = pt.x*sin(angle) + pt.y*cos(angle);
        pt.x = x;
        pt.y = y;
        return pt;
    }

    geometry_msgs::Point createPoint(double x, double y, double z)
    {
        geometry_msgs::Point retPt;
        retPt.x = x;
        retPt.y = y;
        retPt.z = z;
        return retPt;
    }

    void createGoal(double xGoal, double yGoal)
    {
        baseLine.points.clear();
        baseLine.points.push_back(createPoint(robotPose.position.x, robotPose.position.y, 0.));
        baseLine.points.push_back(createPoint(xGoal, yGoal, 0.));

        double dx, dy;
        double angle = getAngle();

        dx = 0.5*cos(angle);
        dy = 0.5*sin(angle);

        geometry_msgs::Point pt1 = createPoint(xGoal - dx, yGoal - dy, 0.);
        geometry_msgs::Point pt2 = createPoint(xGoal + dx, yGoal + dy, 0.);

        dx = (pt2.x+pt1.x)/2.0;
        dy = (pt2.y+pt1.y)/2.0;

        pt1.x -= dx;
        pt1.y -= dy;
        pt1 = rotatePt(pt1, M_PI_2);
        pt1.x += dx;
        pt1.y += dy;

        pt2.x -= dx;
        pt2.y -= dy;
        pt2 = rotatePt(pt2, M_PI_2);
        pt2.x += dx;
        pt2.y += dy;

        goalLine.points.clear();
        goalLine.points.push_back(pt1);
        goalLine.points.push_back(pt2);
    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub1, sub2;
    ros::Publisher cmd_pub, global_goal_pub, local_goal_pub, oa_local_goal_pub, path_goal_pub;
    ros::Time last_time;
    nav_msgs::Odometry last_odom;
    visualization_msgs::Marker goalLine, baseLine;

    bool first, shouldStop;
    double scanVX, scanVY;

    //Params
    geometry_msgs::Pose robotPose;
    double linearVel, rotationVel, goalAcceptedDistance, omegaAcceptedDistance;
    double stopTime;
    double forceGoalVector, fovObstacleAvoidance;
    double minRangeScan, maxRangeScan;
    bool useObstacleAvoidance;
    double maxDist;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "daart_waypoints_line");

    WaypointNAV waypointNAV;
    waypointNAV.run();
}
