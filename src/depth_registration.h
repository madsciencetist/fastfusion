#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/buffer.h>

namespace depth_image_proc
{

  sensor_msgs::ImagePtr imageCb(const sensor_msgs::ImageConstPtr &depth_image_msg,
                                const sensor_msgs::CameraInfoConstPtr &depth_info_msg,
                                const sensor_msgs::CameraInfoConstPtr &rgb_info_msg,
                                const std::string output_encoding,
                                const std::string &fixed_frame,
                                const tf2_ros::Buffer &tf_buffer,
                                float min_depth);

} // end namespace depth_image_proc
