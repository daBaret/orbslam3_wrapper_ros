#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include "orb_slam3_wrapper/image_pos.h"

#include "wrapper_helper.h"

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabImage(const sensor_msgs::ImageConstPtr &msg);

    ORB_SLAM3::System *mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();
    // image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/map_frame_id", map_frame_id, "map");
    node_handler.param<std::string>(node_name + "/pose_frame_id", pose_frame_id, "pose");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file, settings_file, ORB_SLAM3::System::MONOCULAR, enable_pangolin);
    ImageGrabber igb(&SLAM);

    ros::Subscriber sub_img0 = node_handler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    image_pub = node_handler.advertise<orb_slam3_wrapper::image_pos>("/orb_slam3_ros/image_out", 1);

    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped>("/orb_slam3_ros/camera_pose", 1);

    map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3_ros/map_points", 1);

    // setup_tf_orb_to_ros(ORB_SLAM3::System::MONOCULAR);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec()).matrix());

    ros::Time current_frame_time = msg->header.stamp;

    if (cv::sum(Tcw) != cv::Scalar(4))
    {

        publish_ros_tracking_mappoints(mpSLAM->GetTrackedMapPoints(), current_frame_time);

        orb_slam3_wrapper::image_pos image_pos_msg;

        image_pos_msg.header = msg->header;
        image_pos_msg.image = *msg;
        image_pos_msg.pose = publish_ros_pose_tf(Tcw, current_frame_time, ORB_SLAM3::System::MONOCULAR);

        image_pub.publish(image_pos_msg);
    }
}
