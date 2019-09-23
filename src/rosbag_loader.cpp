#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/pass_through.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/message_filter.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/buffer.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>

#include <boost/foreach.hpp>
#include "depth_registration.h"
#include "pointcloud_processing.h"
#include "camerautils/camerautils.hpp"
#include "rosbag_loader.h"

typedef sensor_msgs::CompressedImage RgbMsgT;
typedef sensor_msgs::Image DepthMsgT;

::CameraInfo cameraPoseFromROS(const sensor_msgs::CameraInfoConstPtr &camera_model,
                             const Eigen::Affine3d& T_global_camera)
{
    CameraInfo result;
    cv::Mat intrinsic = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    //Kinect Intrinsic Parameters
    intrinsic.at<double>(0, 0) = camera_model->K[0];
    intrinsic.at<double>(1, 1) = camera_model->K[4];
    intrinsic.at<double>(0, 2) = camera_model->K[2];
    intrinsic.at<double>(1, 2) = camera_model->K[5];

    result.setIntrinsic(intrinsic);
    Eigen::Matrix3d rotation = T_global_camera.rotation();
    cv::Mat rotation2 = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            rotation2.at<double>(i, j) = rotation(i, j);
    result.setRotation(rotation2);
    cv::Mat translation = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    translation.at<double>(0, 0) = T_global_camera.translation().x();
    translation.at<double>(1, 0) = T_global_camera.translation().y();
    translation.at<double>(2, 0) = T_global_camera.translation().z();
    result.setTranslation(translation);
    return result;
}

void imageSyncCallback(const RgbMsgT::ConstPtr &rgb_msg,
                       const DepthMsgT::ConstPtr &depth_msg,
                       const sensor_msgs::CameraInfo::ConstPtr &rgb_camera_info,
                       const sensor_msgs::CameraInfo::ConstPtr &depth_camera_info,
                       const geometry_msgs::TransformStamped::ConstPtr &dummy,
                       float min_depth,
                       tf2_ros::Buffer* tf_buffer,
                       FusionParameter* par,
                       volatile bool *newMesh)
{
    std::string fixed_frame = "p1/map";

    sensor_msgs::ImagePtr registered_depth = depth_image_proc::imageCb(depth_msg,
                                                                       depth_camera_info,
                                                                       rgb_camera_info,
                                                                       sensor_msgs::image_encodings::TYPE_16UC1, // processAsPointCloud and addMap require
                                                                       fixed_frame,
                                                                       *tf_buffer,
                                                                       min_depth);

    cv_bridge::CvImagePtr rgb_cvimg = cv_bridge::toCvCopy(rgb_msg, "rgb8");
    cv_bridge::CvImagePtr depth_cvimg = cv_bridge::toCvCopy(registered_depth);

    Eigen::Affine3d T_global_camera;
    try
    {
        geometry_msgs::TransformStamped transform = tf_buffer->lookupTransform(
            fixed_frame,
            rgb_msg->header.frame_id,
            rgb_msg->header.stamp);
        tf::transformMsgToEigen(transform.transform, T_global_camera);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_DELAYED_THROTTLE(2.0, "no transform to fixed frame");
        return;
    }

    //processAsPointCloud(rgb_cvimg->image, depth_cvimg->image, *rgb_camera_info, T_global_camera);

    ::CameraInfo pose = cameraPoseFromROS(rgb_camera_info, T_global_camera);

    par->fusion->addMap(depth_cvimg->image, pose,
                       rgb_cvimg->image, 1.0f / par->imageDepthScale, par->maxCamDistance);

    *newMesh = par->fusion->updateMeshes();
}

// Load bag
void loadBag(const std::string &filename,
            float min_depth,
            FusionParameter par,
            volatile bool *newMesh,
            volatile bool *fusionActive,
            volatile bool *fusionAlive)
{
    // TODO: make CLI args
    std::string rgb_ns = "eo_camera/forward";
    std::string depth_ns = "depth_camera/forward_down";
    std::string rgb_topic = rgb_ns + "/image_raw/compressed";
    std::string depth_topic = depth_ns + "/image_rect";
    std::string rgb_camera_info_topic = rgb_ns + "/camera_info";
    std::string depth_camera_info_topic = depth_ns + "/camera_info";
    std::string local_pose_topic = "global_map_2d/local_pose";
    std::string tf_topic = "/tf";
    std::string static_tf_topic = "/tf_static";

    rosbag::Bag bag(filename, rosbag::bagmode::Read);

    // Extract static transforms
    tf2_ros::Buffer tf_buffer(ros::Duration(99999.9));
    std::vector<std::string> topics;
    topics.push_back(tf_topic);
    topics.push_back(static_tf_topic);
    rosbag::View static_tf_view(bag, rosbag::TopicQuery(topics));
    BOOST_FOREACH (rosbag::MessageInstance const m, static_tf_view)
    {
        if (*fusionAlive == false)
            return;

        tf2_msgs::TFMessage::ConstPtr tf_msg = m.instantiate<tf2_msgs::TFMessage>();
        if (tf_msg != NULL)
        {
            for (size_t i = 0; i < tf_msg->transforms.size(); ++i)
            {
                bool is_static = (m.getTopic() == static_tf_topic) ||
                                 (tf_msg->transforms[i].child_frame_id.find("submap_") != std::string::npos);

                if (is_static)
                {
                    tf_buffer.setTransform(tf_msg->transforms[i], "", true);
                }
            }
        }
    }

    // Load rest of topics
    topics.clear();
    topics.push_back(rgb_topic);
    topics.push_back(depth_topic);
    topics.push_back(rgb_camera_info_topic);
    topics.push_back(depth_camera_info_topic);
    topics.push_back(local_pose_topic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Set up fake subscribers to capture images. We'll add msgs to them manually.
    message_filters::PassThrough<RgbMsgT> rgb_msg_sub;
    message_filters::PassThrough<DepthMsgT> depth_msg_sub;
    message_filters::PassThrough<sensor_msgs::CameraInfo> rgb_camera_info_sub, depth_camera_info_sub;
    message_filters::PassThrough<geometry_msgs::TransformStamped> tf_is_ready_sub;

    // Synchronize RGB, depth & camera_infos
    ros::Time::init();
    typedef message_filters::sync_policies::ApproximateTime<RgbMsgT,
                                                            DepthMsgT,
                                                            sensor_msgs::CameraInfo,
                                                            sensor_msgs::CameraInfo,
                                                            geometry_msgs::TransformStamped> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy>
        sync(SyncPolicy(30), rgb_msg_sub, depth_msg_sub, rgb_camera_info_sub, depth_camera_info_sub, tf_is_ready_sub);
    sync.registerCallback(boost::bind(&imageSyncCallback, _1, _2, _3, _4, _5, min_depth, &tf_buffer, &par, newMesh));

    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {
        if (*fusionAlive == false)
            return;

        while (*fusionActive == false)
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));

        // RGB image (assume compressed)
        if (m.getTopic() == rgb_topic)
        {
            RgbMsgT::ConstPtr rgb_msg = m.instantiate<RgbMsgT>();
            if (rgb_msg != NULL)
            {
                rgb_msg_sub.add(rgb_msg);
            }
        }

        // Depth image (assume NOT compressed)
        if (m.getTopic() == depth_topic)
        {
            DepthMsgT::ConstPtr depth_msg = m.instantiate<DepthMsgT>();
            if (depth_msg != NULL)
            {
                depth_msg_sub.add(depth_msg);
            }
        }

        // RGB camera info
        if (m.getTopic() == rgb_camera_info_topic)
        {
            sensor_msgs::CameraInfo::ConstPtr cam_info_msg = m.instantiate<sensor_msgs::CameraInfo>();
            if (cam_info_msg != NULL)
            {
                rgb_camera_info_sub.add(cam_info_msg);
            }
        }

        // Depth camera info
        if (m.getTopic() == depth_camera_info_topic)
        {
            sensor_msgs::CameraInfo::ConstPtr cam_info_msg = m.instantiate<sensor_msgs::CameraInfo>();
            if (cam_info_msg != NULL)
            {
                depth_camera_info_sub.add(cam_info_msg);
            }
        }

        // Pose
        if (m.getTopic() == local_pose_topic)
        {
            geometry_msgs::PoseStamped::ConstPtr local_pose_msg = m.instantiate<geometry_msgs::PoseStamped>();
            if (local_pose_msg != NULL)
            {
                geometry_msgs::TransformStamped transform;
                transform.transform.translation.x = local_pose_msg->pose.position.x;
                transform.transform.translation.y = local_pose_msg->pose.position.y;
                transform.transform.translation.z = local_pose_msg->pose.position.z;
                transform.transform.rotation.x = local_pose_msg->pose.orientation.x;
                transform.transform.rotation.y = local_pose_msg->pose.orientation.y;
                transform.transform.rotation.z = local_pose_msg->pose.orientation.z;
                transform.transform.rotation.w = local_pose_msg->pose.orientation.w;
                transform.header = local_pose_msg->header;
                transform.child_frame_id = "p1/base_link";
                tf_buffer.setTransform(transform, "", false);

                // sync this msg so that when the synchronized callback triggers,
                // the transform will be available
                geometry_msgs::TransformStamped::Ptr tf_is_ready_msg = boost::make_shared<geometry_msgs::TransformStamped>();
                tf_is_ready_msg->header.stamp = local_pose_msg->header.stamp - ros::Duration(0.05);
                tf_is_ready_sub.add(tf_is_ready_msg);
            }
        }
    }
}
