#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace ros;

class ScanCopier{

public:
    ScanCopier()
    {
        sub = n.subscribe("/scan", 1, &ScanCopier::onNewScan, this);
        scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan_copy2odom", 50);
    }

    void onNewScan(const sensor_msgs::LaserScan scan_msg)
    {
        sensor_msgs::LaserScan scan_msg2 = scan_msg;
        scan_msg2.header.frame_id = "laser_copy2odom";
        scan_pub.publish(scan_msg2);
    }

private:
    NodeHandle n;
    Subscriber sub;
    Publisher scan_pub;
};

int main(int argc, char **argv)
{
    // Set up ROS.
    init(argc, argv, "scan_copy");
    ScanCopier scanCopier;
    spin();
}
