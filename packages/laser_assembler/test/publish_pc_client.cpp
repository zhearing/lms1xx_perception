#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%

#include <sstream>
using namespace laser_assembler;


ros::Publisher pc_pub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  ros::NodeHandle n;
  ros::service::waitForService("assemble_scans2");
  ros::ServiceClient client = n.serviceClient<AssembleScans2>("assemble_scans2");
  AssembleScans2 srv;

 
  pc_pub = n.advertise<sensor_msgs::PointCloud2>("assemble_pcl",1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {


      srv.request.begin = ros::Time(0,0);
      srv.request.end   = ros::Time::now();
      if (client.call(srv))
      {

        printf("Got cloud with %u points\n", srv.response.cloud.data.size());
        sensor_msgs::PointCloud2 pc = srv.response.cloud;
        pc_pub.publish(pc);
      }
      else
        printf("Service call failed\n");

      ros::spinOnce();

      loop_rate.sleep();
  }
  return 0;
}
