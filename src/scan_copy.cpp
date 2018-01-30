#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

using namespace ros;
using namespace std;

class ScanCopier{

public:
    ScanCopier() : topic2copy("/scan"), topicPub("/scan_copy2odom"), frameId("laser_copy2odom")
    {
        ros::NodeHandle nodeLocal("~");

        topic2copy = nodeLocal.param("topic2copy", topic2copy);
        topicPub = nodeLocal.param("topicPub", topicPub);
        frameId = nodeLocal.param("frameId", frameId);

        sub = n.subscribe(topic2copy, 1, &ScanCopier::onNewScan, this);
        scan_pub = n.advertise<sensor_msgs::LaserScan>(topicPub, 50);
    }

    void onNewScan(const sensor_msgs::LaserScan scan_msg)
    {
        sensor_msgs::LaserScan scan_msg2 = scan_msg;
        scan_msg2.header.frame_id = frameId;
        scan_pub.publish(scan_msg2);
    }

private:
    NodeHandle n;
    Subscriber sub;
    Publisher scan_pub;
    string topic2copy, topicPub, frameId;
};

int main(int argc, char **argv)
{
    // Set up ROS.
    init(argc, argv, "scan_copy");
    ScanCopier scanCopier;
    spin();
}
