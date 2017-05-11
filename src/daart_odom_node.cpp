#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include "I2CIO.h"
// #include <crc.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


const uint8_t crc_table[256] = {
    0x00U,0x5EU,0xBCU,0xE2U,0x61U,0x3FU,0xDDU,0x83U,
    0xC2U,0x9CU,0x7EU,0x20U,0xA3U,0xFDU,0x1FU,0x41U,
    0x9DU,0xC3U,0x21U,0x7FU,0xFCU,0xA2U,0x40U,0x1EU,
    0x5FU,0x01U,0xE3U,0xBDU,0x3EU,0x60U,0x82U,0xDCU,
    0x23U,0x7DU,0x9FU,0xC1U,0x42U,0x1CU,0xFEU,0xA0U,
    0xE1U,0xBFU,0x5DU,0x03U,0x80U,0xDEU,0x3CU,0x62U,
    0xBEU,0xE0U,0x02U,0x5CU,0xDFU,0x81U,0x63U,0x3DU,
    0x7CU,0x22U,0xC0U,0x9EU,0x1DU,0x43U,0xA1U,0xFFU,
    0x46U,0x18U,0xFAU,0xA4U,0x27U,0x79U,0x9BU,0xC5U,
    0x84U,0xDAU,0x38U,0x66U,0xE5U,0xBBU,0x59U,0x07U,
    0xDBU,0x85U,0x67U,0x39U,0xBAU,0xE4U,0x06U,0x58U,
    0x19U,0x47U,0xA5U,0xFBU,0x78U,0x26U,0xC4U,0x9AU,
    0x65U,0x3BU,0xD9U,0x87U,0x04U,0x5AU,0xB8U,0xE6U,
    0xA7U,0xF9U,0x1BU,0x45U,0xC6U,0x98U,0x7AU,0x24U,
    0xF8U,0xA6U,0x44U,0x1AU,0x99U,0xC7U,0x25U,0x7BU,
    0x3AU,0x64U,0x86U,0xD8U,0x5BU,0x05U,0xE7U,0xB9U,
    0x8CU,0xD2U,0x30U,0x6EU,0xEDU,0xB3U,0x51U,0x0FU,
    0x4EU,0x10U,0xF2U,0xACU,0x2FU,0x71U,0x93U,0xCDU,
    0x11U,0x4FU,0xADU,0xF3U,0x70U,0x2EU,0xCCU,0x92U,
    0xD3U,0x8DU,0x6FU,0x31U,0xB2U,0xECU,0x0EU,0x50U,
    0xAFU,0xF1U,0x13U,0x4DU,0xCEU,0x90U,0x72U,0x2CU,
    0x6DU,0x33U,0xD1U,0x8FU,0x0CU,0x52U,0xB0U,0xEEU,
    0x32U,0x6CU,0x8EU,0xD0U,0x53U,0x0DU,0xEFU,0xB1U,
    0xF0U,0xAEU,0x4CU,0x12U,0x91U,0xCFU,0x2DU,0x73U,
    0xCAU,0x94U,0x76U,0x28U,0xABU,0xF5U,0x17U,0x49U,
    0x08U,0x56U,0xB4U,0xEAU,0x69U,0x37U,0xD5U,0x8BU,
    0x57U,0x09U,0xEBU,0xB5U,0x36U,0x68U,0x8AU,0xD4U,
    0x95U,0xCBU,0x29U,0x77U,0xF4U,0xAAU,0x48U,0x16U,
    0xE9U,0xB7U,0x55U,0x0BU,0x88U,0xD6U,0x34U,0x6AU,
    0x2BU,0x75U,0x97U,0xC9U,0x4AU,0x14U,0xF6U,0xA8U,
    0x74U,0x2AU,0xC8U,0x96U,0x15U,0x4BU,0xA9U,0xF7U,
    0xB6U,0xE8U,0x0AU,0x54U,0xD7U,0x89U,0x6BU,0x35U,
    };

uint8_t crc8(unsigned char* data, int len, uint8_t crc){

  /*
    Automatically generated CRC function
    polynomial: 0x131, bit reverse algorithm
    0x131 == Dallas polynom, generated with python crcmod

    import crcmod
    fd = open("foo.c", "a")
    crc8 = crcmod.Crc(0x131) #Dallas polynom
    crc8.generateCode("crc8", fd)
  */


    while (len > 0)
    {
        crc = crc_table[*data ^ (uint8_t)crc];
        data++;
        len--;
    }

    return crc;
}


int file;

void openConnectionTREX()
{
    char *filename = "/dev/i2c-2";
    int addr = 0x07;

    if ( (file = open(filename,O_RDWR)) < 0 )
    {
       ROS_ERROR("Failed to open i2c bus. Shutdown.");
       ros::shutdown();
    }

    ioctl(file,I2C_TENBIT, 0);

    if ( ioctl(file,I2C_SLAVE, addr) < 0 )
    {
       ROS_ERROR("Failed to talk to T-Rex. Shutdown.");
       ros::shutdown();
    }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "daart_odom_node");
  openConnectionTREX();
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  int left_encoder_prev = 0;
  int right_encoder_prev = 0;
  double wheelsDistance = 10.2;
  double wheelsDiameter = 16.2;
  double rate = 1.0/30000.;
  bool firstReading = true;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();


   //Read from T-Rex
   I2C_output_packet recv;
   uint8_t crc = 0;
   if (read(file,&recv,sizeof(I2C_output_packet)) != sizeof(I2C_output_packet))
   {
      ROS_ERROR("Cannot read bytes");
   }
   else
   {
      crc = crc8((uint8_t*)&recv, sizeof(I2C_output_packet)-1, 0);
      ROS_DEBUG("expected crc: %hhu\n", crc);
      ROS_DEBUG("got: %hhu\n", recv.crc);

      ROS_DEBUG("left motor encoder = %hd\n",recv.left_encoder);
      ROS_DEBUG("right motor encoder = %hd\n",recv.right_encoder);
      if(firstReading)
      {
        firstReading = false;
        left_encoder_prev = recv.left_encoder;
        right_encoder_prev = recv.right_encoder;
      }

      double LED = rate*wheelsDiameter*(recv.left_encoder - left_encoder_prev);
      double RED = rate*wheelsDiameter*(recv.right_encoder - right_encoder_prev);

      double meanDistance = (LED+RED)/2.0;


      //compute odometry in a typical way given the velocities of the robot
      double dt = (current_time - last_time).toSec();
      double dx = meanDistance*cos(th);
      double dy = meanDistance*sin(th);
      double omega = (RED-LED)/wheelsDistance;

      x += dx;
      y += dy;
      th += omega;

      double vx = meanDistance / dt;
      double vy = 0;
      omega /= dt;

      left_encoder_prev = recv.left_encoder;
      right_encoder_prev = recv.right_encoder;

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = omega;

      //publish the message
      odom_pub.publish(odom);

      last_time = current_time;
      r.sleep();
    }
  }
}
