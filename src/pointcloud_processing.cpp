#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include "pointcloud_processing.h"

pcl::PointCloud<pcl::PointXYZRGB> full_cloud;

void processAsPointCloud(const cv::Mat &depth_img,
                         const cv::Mat &rgb_img,
                         const sensor_msgs::CameraInfo camera_info,
                         Eigen::Affine3d T_global_camera)
{


    double fx = camera_info.K[0];
    double fy = camera_info.K[4];
    double cx = camera_info.K[2];
    double cy = camera_info.K[5];

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    full_cloud.width += depth_img.rows;
    full_cloud.height = 1;
    full_cloud.resize(full_cloud.width * full_cloud.height);

    for (int row = 0; row < depth_img.rows; row++)
    {
        for (int col = 0; col < depth_img.cols; col++)
        {
            ushort z = depth_img.at<ushort>(row, col);
            if (z > 0 && std::isfinite(z))
            {
                cv::Vec3b rgb = rgb_img.at<cv::Vec3b>(row, col);
                pcl::PointXYZRGB pt(rgb[0], rgb[1], rgb[2]);

                // Transform to global frame
                Eigen::Vector4d xyz_camera;
                xyz_camera << (col - cx) / fx * z,
                    (row - cy) / fy * z,
                    z,
                    1;

                Eigen::Vector4d xyz_global = T_global_camera * xyz_camera;
                pt.x = xyz_global[0];
                pt.y = xyz_global[1];
                pt.z = xyz_global[2];

                full_cloud.push_back(pt);
            }
        }
    }
}
