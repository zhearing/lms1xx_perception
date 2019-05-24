#include <cloud_merger/cloud_merger.h>

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "CloudMerger");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(25);


    // Create a ROS publisher for the output point cloud
    CloudMerger::CloudMerger *cm = new CloudMerger::CloudMerger(nh, private_nh, it);

    // Spin
    while (ros::ok()) {
        cm->mergeNpub();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // Create a container for the data.
}
