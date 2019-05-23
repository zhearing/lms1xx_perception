#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//#include <turtlesim/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

const double install_x_Roll = -90.0/180.0*3.1415926;
double  first_timestam_sec ;
bool isFirstScanComing = true;



void scanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan)
  {
    if(isFirstScanComing)
    {
        first_timestam_sec = laser_scan->header.stamp.toSec();
        isFirstScanComing = false;
    }

      static tf::TransformBroadcaster br;

      double cur_timestamp_sec = laser_scan->header.stamp.toSec();

      double pitch = (cur_timestamp_sec - first_timestam_sec)*40;
      pitch = -(pitch)/180.0*3.1415926;

      tf::Quaternion q;
      q.setRPY(install_x_Roll,0,pitch);
      tf::Transform laser_transform(q);


      // 

      br.sendTransform(tf::StampedTransform(laser_transform, laser_scan->header.stamp, "world", "laser"));

  }


int main(int argc, char** argv){
  ros::init(argc, argv, "laser_tf_broadcaster");
 

  ros::NodeHandle node;
  ros::Subscriber scan_sub_ = node.subscribe("scan", 10, scanCallback);

  ros::spin();
  return 0;
};
