#include <opencv2/core.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Geometry>

void processAsPointCloud(const cv::Mat &depth_img,
                         const cv::Mat &rgb_img,
                         const sensor_msgs::CameraInfo camera_info,
                         Eigen::Affine3d T_global_camera);
