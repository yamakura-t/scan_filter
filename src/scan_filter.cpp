#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int32.h>

#include <iostream>

class ScanFilter
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;
  ros::Rate loop_rate;

  // 他パッケージの都合上(cmd_vel -> cmd_vel_filter)の順番にしました.
  ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("topurg_front_filtered/scan", 10);
  ros::Subscriber sub = nh.subscribe("topurg_front/scan", 1, &ScanFilter::LaserScanSubCallback, this);

  sensor_msgs::LaserScan scan_base;
  sensor_msgs::LaserScan scan_filtered;
  int maxRange;    //[m]
  int flag = false;

public:
  ScanFilter(
      const ros::NodeHandle &nh_,
      const ros::NodeHandle &nh_private_,
      const ros::Rate &loop_rate_) : nh{nh_}, nh_private{nh_private_}, loop_rate{loop_rate_} {}


  void LaserScanSubCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    scan_base = *msg;
    flag = true;
  }

  void setMaxRange(int range)
  {
    maxRange = range;
  }

  void filter(void)
  {
    scan_filtered = scan_base;

    int step = std::round((scan_base.angle_max - scan_base.angle_min)/scan_base.angle_increment) + 1;

    for (int i = 0; i < step; i++)
    {
      if (!std::isfinite(scan_base.ranges[i]))
      {
        scan_filtered.ranges[i] = maxRange;
      }
      else if (scan_base.ranges[i] >= maxRange)
      {
        scan_filtered.ranges[i] = maxRange;
      }
      else
      {
        scan_filtered.ranges[i] = scan_base.ranges[i];
      }
    }
  }

  void start(void)
  {
    while (ros::ok())
    {
      ros::spinOnce();

    if(flag){
      filter();
      sensor_msgs::LaserScan msg = scan_filtered;

      pub.publish(msg);
    }

      loop_rate.sleep();
    }
  };
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "scan_filter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Rate loop_rate(50); // Hz

  ScanFilter filter(nh, nh_private, loop_rate);
  filter.setMaxRange(30);
  filter.start();

  ros::shutdown();

  return 0;
}