git@github.com:inhok996/ROS_PointMsg_Merger.git

# ROS_PointMsg_Merger
For the robots which have more than two pointcloud-related sensors, this ROS package has the node that can merge messages from sensor_msg::PointCloud2 topics and publish one output topic.(to virtual frame)

If you want to use multiple sensors as if only one sensor exists, this package is right choice.

Since this package works for only locationally fixed sensors, You must know where your sensors are located on your robot.(base_link)

Install: download or clone this package to your catkin workspace and just do catkin_make.


Usage :
  1. Since You may have your sensor drivers like velodyne_driver to collect data through sensor_msg::PointCloud2 topics,
  what you have to do fist is that you register your sensor infos to config/Clouds.yaml.(CloudIn1~, in order,minimum: 1,maximum: 8)
  To do this, you might have to know exact location of the sensors from your robot.
  
  2. Make virtual sensor. I I used static_transform_publisher in launch/Starter.launch. This virtual sensor will be used instead of
     your multiple sensors. Register this virtual sensor information to config/Clouds.yaml.(CloudOut)
  
  3. That's it! Just launch launch/Starter.launch and see if your pointCloud datas are merged.
  
  
