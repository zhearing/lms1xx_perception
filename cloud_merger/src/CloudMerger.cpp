#include<cloud_merger/cloud_merger.h>

namespace CloudMerger {

//InputCloud member function
    InputCloud::InputCloud(pose p, std::string topic, ros::NodeHandle nh) {
        //initialize InputCloud
        this->ps = p;
        this->topic_name = topic;
        this->transform = poseTotfmatrix();
        sub = nh.subscribe(topic_name, 1, &InputCloud::cloudCallback, this);
    }

    void InputCloud::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input) {
        pcl::fromROSMsg(*input, this->inCloud);
        pcl::transformPointCloud(this->inCloud, this->tfdinCloud, transform);
    }

    Eigen::Matrix4f InputCloud::poseTotfmatrix() {
        Eigen::Matrix4f tfMatrix = Eigen::Matrix4f::Identity();
        Eigen::AngleAxisd rollAngle(this->ps.roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(this->ps.pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(this->ps.yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;

        Eigen::Matrix3d rotationMatrix = q.matrix();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                tfMatrix(i, j) = rotationMatrix(i, j);

        //transform x,y,z (gap from virtual frame)
        tfMatrix(0, 3) = this->ps.x;
        tfMatrix(1, 3) = this->ps.y;
        tfMatrix(2, 3) = this->ps.z;

        std::cout << "tfMatrix" << std::endl;
        std::cout << tfMatrix << std::endl;
        return tfMatrix;

    }

//InputCloud member function end

//OutputCloud member function start
    OutputCloud::OutputCloud(std::string topic, std::string frame, ros::NodeHandle nh,
                             image_transport::ImageTransport it) {
        this->topic_name = topic;
        this->frame_id = frame;
        pub = nh.advertise<sensor_msgs::PointCloud2>(this->topic_name, 1);
        this->outCloud.header.frame_id = this->frame_id;
        obs_image_pub = it.advertise("LMS151_FUSION", 1);
    }

//OutputCloud member function end

//CloudMerger Member function
    CloudMerger::CloudMerger(ros::NodeHandle node, ros::NodeHandle private_nh, image_transport::ImageTransport it) {
        //use private node handle to get parameters
        std::string s_key("CloudIn0"); //searching key (input)
        std::string so_key("CloudOut"); //searching key (output)
        std::string key;
        this->nsensors = 0;

        //get all parameters
        for (int i = 1; i <= MAX_NSENSORS; i++) {
            //Searching key must be Cloud[1] ~ Cloud[MAX_NSENSORS]
            s_key[s_key.find((i + '0') - 1)] = i + '0';
            if (private_nh.searchParam(s_key, key)) {
                std::string topic_name(s_key); //set to default
                pose cloud_pose = {0, 0, 0, 0, 0}; //set to default

                if (!private_nh.getParam(key + "/topic_name", topic_name)) {
                    std::cout << "not found : " << key + "/topic_name" << std::endl;
                }
                if (!private_nh.getParam(key + "/pose_yaw", cloud_pose.yaw)) {
                    std::cout << "not found : " << key + "/pose_yaw" << std::endl;
                }
                if (!private_nh.getParam(key + "/pose_pitch", cloud_pose.pitch)) {
                    std::cout << "not found : " << key + "/pose_pitch" << std::endl;
                }
                if (!private_nh.getParam(key + "/pose_roll", cloud_pose.roll)) {
                    std::cout << "not found : " << key + "/pose_roll" << std::endl;
                }
                if (!private_nh.getParam(key + "/pose_x", cloud_pose.x)) {
                    std::cout << "not found : " << key + "/pose_x" << std::endl;
                }
                if (!private_nh.getParam(key + "/pose_y", cloud_pose.y)) {
                    std::cout << "not found : " << key + "/pose_y" << std::endl;
                }
                if (!private_nh.getParam(key + "/pose_z", cloud_pose.z)) {
                    std::cout << "not found : " << key + "/pose_z" << std::endl;
                }
                cloud_pose.yaw = M_PI * (cloud_pose.yaw / 180.0);//degree to radian
                cloud_pose.pitch = M_PI * (cloud_pose.pitch / 180.0);//degree to radian
                cloud_pose.roll = M_PI * (cloud_pose.roll / 180.0);//degree to radian
                std::cout << key << " info" << std::endl;
                std::cout << "(yaw,pitch,roll,x,y,z) = "
                          << cloud_pose.yaw << " "
                          << cloud_pose.pitch << " "
                          << cloud_pose.roll << " "
                          << cloud_pose.x << " "
                          << cloud_pose.y << " "
                          << cloud_pose.z << std::endl;
                //create inputCloud object
                this->nsensors++;
                inClAry[i - 1] = new InputCloud(cloud_pose, topic_name, node);
            }
        }
        if (this->nsensors == 0) std::cout << "No input data found" << std::endl;
        //outputCloud object
        if (private_nh.searchParam(so_key, key)) {
            std::string topic_name(so_key); //set to default
            std::string frame_id("default_frame_id"); //set to default
            if (!private_nh.getParam(key + "/topic_name", topic_name)) {
                std::cout << "not found : " << key + "/topic_name" << std::endl;
            }
            if (!private_nh.getParam(key + "/frame_id", frame_id)) {
                std::cout << "not found : " << key + "/frame_id" << std::endl;
            }
            outCl = new OutputCloud(topic_name, frame_id, node, it);
        }
    }

    sensor_msgs::ImagePtr CloudMerger::constructObstacleMap(const pcl::PointCloud<pcl::PointXYZ> &scan) {
        int grid_dim_x = 500;
        int grid_dim_y = 750;
        float m_per_cell_ = 0.2;
        cv::Mat obs_image = cv::Mat::zeros(grid_dim_y, grid_dim_x, CV_8UC1);

        for (size_t i = 0; i < scan.points.size(); ++i) {
            int y = ((grid_dim_y * 2 / 3) - scan.points[i].x / m_per_cell_);
            int x = ((grid_dim_x / 2) - scan.points[i].y / m_per_cell_);
            obs_image.ptr<char>(y)[x] = 255;
        }
        sensor_msgs::ImagePtr obs_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",
                                                                 obs_image).toImageMsg();
        return obs_image_msg;
    }

    void CloudMerger::mergeNpub() {
        /* synchronization problem
         * Time stamp of 4 sensors are apparently different.
         * velodyne drivers publish messages every 100ms(10hz)
         * Is it ok?
         */
        outCl->outCloud.clear(); //clear before use. Header info is not cleared.
        for (int i = 0; i < this->nsensors; i++) {
            outCl->outCloud += inClAry[i]->tfdinCloud;
        }

        //initialize header info with first Cloud info
        outCl->outCloud.header.seq = inClAry[0]->tfdinCloud.header.seq;
        outCl->outCloud.header.stamp = inClAry[0]->tfdinCloud.header.stamp;
        pcl::toROSMsg(outCl->outCloud, outCl->outCloudMsg);
        outCl->pub.publish(outCl->outCloudMsg);
        sensor_msgs::ImagePtr obs_image_msg = constructObstacleMap(outCl->outCloud);
        outCl->obs_image_pub.publish(obs_image_msg);
    }
//CloudMerger member function end

}//namespace of CloudMerger
