#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include <signal.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include "I2CIO.h"
// #include "crc.h"

using namespace std;

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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}



int file;
double wheelsDistance = 0.255;
double wheelsDiameter = 0.40;
double minVel = 2.5, maxVel = 3;
double shiftMoving = 0, shiftInPlace = 0.0;
double scaleMoving = 1.0, scaleInPlace = 1.0;

double v1Desired = 0.0, v2Desired = 0.0, v1 = 0.0, v2 = 0.0;
double bumping = false;

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

void sendVel2TREX(double v1, double v2)
{
    I2C_input_packet to_send;
    bzero(&to_send, sizeof(I2C_input_packet));

    double rate = 0.1777777778;
    to_send.left_motor_speed  = round(5.*v1/rate +10.*sgn(v1));
    to_send.right_motor_speed = round(5.*v2/rate +10.*sgn(v2));
    if (v1 == 0.0)
      to_send.left_motor_speed  = 0;
    if (v2 == 0.0)
      to_send.right_motor_speed  = 0;


    to_send.crc = crc8((unsigned char*) &to_send, sizeof(I2C_input_packet)-1, 0);

    if (write(file, &to_send, sizeof(I2C_input_packet)) != sizeof(I2C_input_packet))
       ROS_ERROR("Cannot write bytes");
    ROS_INFO("Sent.");
}

void processTwist(const geometry_msgs::Twist vel_msg)
{
  double vel = vel_msg.linear.x;
  double omega = vel_msg.angular.z;

  if (omega != 0.0)
  {
      if(vel_msg.linear.x == 0.0)
        omega = omega*scaleInPlace + shiftInPlace*sgn(omega);
      else
        omega = omega*scaleMoving + shiftMoving*sgn(omega);
  }

  stringstream msgStream;
  msgStream << "Vel: " << vel << "\t Omega:" << omega;
  ROS_INFO(msgStream.str().c_str());

  v2 = (vel*2 + omega*wheelsDistance) /2.0;
  v1 = vel*2  - v2;

  v1 = min(v1, maxVel);
  v2 = min(v2, maxVel);

  if (v1 != 0.0)
    v1 = max(v1*sgn(v1), minVel)*sgn(v1);

  if (v2 != 0.0)
    v2 = max(v2*sgn(v2), minVel)*sgn(v2);
  // if (v1Desired ==0.0 || v2Desired ==0.0)
  // {
  //   sendVel2TREX(1.2, 1.2);
  //   usleep(500);
  // }
  v1Desired = v1;
  v2Desired = v2;
  bumping = false;
  sendVel2TREX(v1, v2);
  cout << v1 << " - " << v2 << endl;
}

geometry_msgs::Twist old_vel_msg;

void velCallback(const geometry_msgs::Twist vel_msg)
{
  old_vel_msg = vel_msg;
  if (bumping)
  {
    return;
  }
  processTwist(vel_msg);
}

void odomCallback(const nav_msgs::Odometry odom)
{
  bool reset = true;
  if( v1Desired == 0.0 && v2Desired == 0.0)
  {
    return;
  }

  if (odom.twist.twist.linear.x == 0.0 && v1Desired != -v2Desired && v1Desired != 0.0 && v2Desired != 0.0)
  {
      bumping = true;
      reset = false;
      v1 = v1 + 1*sgn(v1);
      v2 = v2 + 1*sgn(v2);
      v1 = min(v1, maxVel);
      v2 = min(v2, maxVel);
      sendVel2TREX(v1, v2);
      ROS_INFO("Vel: %f", odom.twist.twist.linear.x);
      ROS_INFO("Bumping, v1: %f, v2: %f",v1, v2);
  }

  if (odom.twist.twist.angular.z == 0.0 && v1Desired != v2Desired)
  {
      bumping = true;
      reset = false;
      // if(sgn(v1) != sgn(v2))
      // {
      //   double oldD = fabs(v2 - v1);
      //   double newD = oldD+1;
      //   v1 = newD*( v1/oldD );
      //   v2 = newD*( v2/oldD );
      // }
      // else
      if(fabs(v1Desired) > fabs(v2Desired))
         v1 = v1 + 1*sgn(v1);
      else
         v2 = v2 + 1*sgn(v2);

      v1 = min(v1, maxVel);
      v2 = min(v2, maxVel);
      sendVel2TREX(v1, v2);
      ROS_INFO("Omega: %f", odom.twist.twist.angular.z);
      ROS_INFO("Bumping, v1: %f, v2: %f",v1, v2);
  }

  if(reset)
  {
      if(bumping)
      {
        bumping = false;
        processTwist(old_vel_msg);
      }
  }
}

void shuttingdown(int signal)
{
  sendVel2TREX(0.0, 0.0);
  ROS_INFO("Speed control shutting down");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "daart_speed_control_node");
  openConnectionTREX();

  ros::NodeHandle n;

  ROS_INFO("Hello world!");
  std::string ns = ros::this_node::getNamespace();
  ros::Subscriber sub1 = n.subscribe(ns+"/cmd_vel", 0, velCallback);
  ros::Subscriber sub2 = n.subscribe(ns+"/odom", 0, odomCallback);

  signal(SIGINT, shuttingdown);
  ros::spin();
  return 0;
}
