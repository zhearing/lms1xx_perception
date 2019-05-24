#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>

class LaserToCloud {
public:
    //typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    //bool cloud_completed = false;
    LaserToCloud();

    void front_scanner_callback(const sensor_msgs::LaserScan::ConstPtr &front_scan);

    void left_scanner_callback(const sensor_msgs::LaserScan::ConstPtr &left_scan);

    void right_scanner_callback(const sensor_msgs::LaserScan::ConstPtr &right_scan);

    void rear_scanner_callback(const sensor_msgs::LaserScan::ConstPtr &rear_scan);

private:
    ros::NodeHandle nh;
    laser_geometry::LaserProjection projector;
    tf::TransformListener tfListener;

    ros::Subscriber front_scanner_sub;
    ros::Subscriber left_scanner_sub;
    ros::Subscriber right_scanner_sub;
    ros::Subscriber rear_scanner_sub;

    ros::Publisher pub_cloud_front_;
    ros::Publisher pub_cloud_left_;
    ros::Publisher pub_cloud_right_;
    ros::Publisher pub_cloud_rear_;
    tf::TransformListener listener;

};


LaserToCloud::LaserToCloud() {
    front_scanner_sub = nh.subscribe<sensor_msgs::LaserScan>("front_scan", 10, &LaserToCloud::front_scanner_callback,
                                                             this);
    left_scanner_sub = nh.subscribe<sensor_msgs::LaserScan>("left_scan", 10, &LaserToCloud::left_scanner_callback,
                                                            this);
    right_scanner_sub = nh.subscribe<sensor_msgs::LaserScan>("right_scan", 10, &LaserToCloud::right_scanner_callback,
                                                             this);
    rear_scanner_sub = nh.subscribe<sensor_msgs::LaserScan>("rear_scan", 10, &LaserToCloud::rear_scanner_callback,
                                                            this);
    //front_scanner_sub = nh.subscribe<sensor_msgs::LaserScan>("front_scan", 10, &LaserToCloud::front_scanner_callback, this);
    //left_scanner_sub = nh.subscribe<sensor_msgs::LaserScan>("left_scan", 10, &LaserToCloud::left_scanner_callback, this);
    //right_scanner_sub = nh.subscribe<sensor_msgs::LaserScan>("right_scan", 10, &LaserToCloud::right_scanner_callback, this);
    //rear_scanner_sub = nh.subscribe<sensor_msgs::LaserScan>("rear_scan", 10, &LaserToCloud::rear_scanner_callback, this);
    pub_cloud_front_ = nh.advertise<sensor_msgs::PointCloud2>("pcl_front", 10, false);
    pub_cloud_left_ = nh.advertise<sensor_msgs::PointCloud2>("pcl_left", 10, false);
    pub_cloud_right_ = nh.advertise<sensor_msgs::PointCloud2>("pcl_right", 10, false);
    pub_cloud_rear_ = nh.advertise<sensor_msgs::PointCloud2>("pcl_rear", 10, false);

    //tfListener.setExtrapolationLimit(ros::Duration(0.1));
}

void LaserToCloud::front_scanner_callback(const sensor_msgs::LaserScan::ConstPtr &front_scan) {
    if (!listener.waitForTransform(front_scan->header.frame_id, "/tramcar", front_scan->header.stamp +
                                                                         ros::Duration().fromSec(
                                                                                 front_scan->ranges.size() *
                                                                                 front_scan->time_increment),
                                   ros::Duration(1.0))) {
        return;

    }

    sensor_msgs::PointCloud2 cloud1;

    projector.transformLaserScanToPointCloud("/tramcar", *front_scan, cloud1, listener);


    // Do something with cloud.






    pub_cloud_front_.publish(cloud1);

}

void LaserToCloud::left_scanner_callback(const sensor_msgs::LaserScan::ConstPtr &left_scan) {
    if (!listener.waitForTransform(left_scan->header.frame_id, "/tramcar", left_scan->header.stamp +
                                                                        ros::Duration().fromSec(
                                                                                left_scan->ranges.size() *
                                                                                left_scan->time_increment),
                                   ros::Duration(1.0))) {
        return;

    }

    sensor_msgs::PointCloud2 cloud2;
    projector.transformLaserScanToPointCloud("tramcar", *left_scan, cloud2, listener);

    // Do something with cloud.

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(cloud2, *cloud);
    pcl::PCLPointCloud2::Ptr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Perform the actual filtering
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    // build the filter
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.3, 5);
    // apply filter
    pass.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(cloud_filtered, output);

    // Publish the data
    pub_cloud_left_.publish(output);

}

void LaserToCloud::right_scanner_callback(const sensor_msgs::LaserScan::ConstPtr &right_scan) {

    if (!listener.waitForTransform(right_scan->header.frame_id, "/tramcar", right_scan->header.stamp +
                                                                         ros::Duration().fromSec(
                                                                                 right_scan->ranges.size() *
                                                                                 right_scan->time_increment),
                                   ros::Duration(1.0))) {
        return;

    }

    sensor_msgs::PointCloud2 cloud3;

    projector.transformLaserScanToPointCloud("tramcar", *right_scan, cloud3, listener);

    // Do something with cloud.

    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(cloud3, *cloud);
    pcl::PCLPointCloud2::Ptr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;


    // Perform the actual filtering
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    // build the filter
    pass.setInputCloud (cloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.3, 5);
    // apply filter
    pass.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_pt;
    pcl_conversions::moveFromPCL(cloud_filtered, cloud_pt);

    // Publish the data
    pub_cloud_right_.publish(cloud_pt);

}

void LaserToCloud::rear_scanner_callback(const sensor_msgs::LaserScan::ConstPtr &rear_scan) {

    if (!listener.waitForTransform(rear_scan->header.frame_id, "/tramcar", rear_scan->header.stamp +
                                                                        ros::Duration().fromSec(
                                                                                rear_scan->ranges.size() *
                                                                                rear_scan->time_increment),
                                   ros::Duration(1.0))) {
        return;

    }

    sensor_msgs::PointCloud2 cloud4;

    projector.transformLaserScanToPointCloud("tramcar", *rear_scan, cloud4, listener);


    // Do something with cloud.

    pub_cloud_rear_.publish(cloud4);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_to_cloud");
    LaserToCloud lasertocloud;
    ros::spin();

}
