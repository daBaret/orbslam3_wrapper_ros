#include "wrapper_helper.h"

ros::Publisher pose_pub;
ros::Publisher map_points_pub;
ros::Publisher image_pub;

std::string map_frame_id, pose_frame_id;

tf::Matrix3x3 tf_orb_to_ros(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);

tf::Transform from_orb_to_ros_tf_transform(cv::Mat transformation_mat)
{
    cv::Mat orb_rotation(3, 3, CV_32F);
    cv::Mat orb_translation(3, 1, CV_32F);

    orb_rotation = transformation_mat.rowRange(0, 3).colRange(0, 3);
    orb_translation = transformation_mat.rowRange(0, 3).col(3);

    tf::Matrix3x3 tf_camera_rotation(
        orb_rotation.at<float>(0, 0), orb_rotation.at<float>(0, 1), orb_rotation.at<float>(0, 2),
        orb_rotation.at<float>(1, 0), orb_rotation.at<float>(1, 1), orb_rotation.at<float>(1, 2),
        orb_rotation.at<float>(2, 0), orb_rotation.at<float>(2, 1), orb_rotation.at<float>(2, 2));

    tf::Vector3 tf_camera_translation(orb_translation.at<float>(0), orb_translation.at<float>(1), orb_translation.at<float>(2));

    // cout << setprecision(9) << "Rotation: " << endl << orb_rotation << endl;
    // cout << setprecision(9) << "Translation xyz: " << orb_translation.at<float> (0) << " " << orb_translation.at<float> (1) << " " << orb_translation.at<float> (2) << endl;

    // Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    // Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf::Transform(tf_camera_rotation, tf_camera_translation);
}

geometry_msgs::PoseStamped publish_ros_pose_tf(cv::Mat Tcw, ros::Time current_frame_time, ORB_SLAM3::System::eSensor sensor_type)
{
    geometry_msgs::PoseStamped pose_msg;
    if (!Tcw.empty())
    {
        tf::Transform tf_transform = from_orb_to_ros_tf_transform(Tcw);

        publish_tf_transform(tf_transform, current_frame_time);

        pose_msg = publish_pose_stamped(tf_transform, current_frame_time);
    }
    return pose_msg;
}

void publish_tf_transform(tf::Transform tf_transform, ros::Time current_frame_time)
{
    static tf::TransformBroadcaster tf_broadcaster;

    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, current_frame_time, map_frame_id, pose_frame_id));
}

geometry_msgs::PoseStamped publish_pose_stamped(tf::Transform tf_transform, ros::Time current_frame_time)
{
    tf::Stamped<tf::Pose> grasp_tf_pose(tf_transform, current_frame_time, map_frame_id);

    geometry_msgs::PoseStamped pose_msg;

    tf::poseStampedTFToMsg(grasp_tf_pose, pose_msg);

    pose_pub.publish(pose_msg);

    return pose_msg;
}

void publish_ros_tracking_mappoints(std::vector<ORB_SLAM3::MapPoint *> map_points, ros::Time current_frame_time)
{
    sensor_msgs::PointCloud2 cloud = tracked_mappoints_to_pointcloud(map_points, current_frame_time);

    map_points_pub.publish(cloud);
}

sensor_msgs::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points, ros::Time current_frame_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = current_frame_time;
    cloud.header.frame_id = map_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {

            tf::Vector3 point_translation(map_points[i]->GetWorldPos().x(), map_points[i]->GetWorldPos().y(), map_points[i]->GetWorldPos().z());

            point_translation = tf_orb_to_ros * point_translation;

            float data_array[num_channels] = {point_translation.x(), point_translation.y(), point_translation.z()};

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
        }
    }
    return cloud;
}