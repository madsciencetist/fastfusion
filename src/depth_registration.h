#include <depth_image_proc/depth_traits.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>

namespace depth_image_proc
{

using namespace message_filters::sync_policies;
namespace enc = sensor_msgs::image_encodings;

template <typename T>
void convert(const sensor_msgs::ImageConstPtr &depth_msg,
             const sensor_msgs::ImagePtr &registered_msg,
             const Eigen::Affine3d &depth_to_rgb,
             const image_geometry::PinholeCameraModel &depth_model,
             const image_geometry::PinholeCameraModel &rgb_model)
{
  // Allocate memory for registered depth image
  registered_msg->step = registered_msg->width * sizeof(T);
  registered_msg->data.resize(registered_msg->height * registered_msg->step);
  // data is already zero-filled in the uint16 case, but for floats we want to initialize everything to NaN.
  DepthTraits<T>::initializeBuffer(registered_msg->data);

  // Extract all the parameters we need
  double inv_depth_fx = 1.0 / depth_model.fx();
  double inv_depth_fy = 1.0 / depth_model.fy();
  double depth_cx = depth_model.cx(), depth_cy = depth_model.cy();
  double depth_Tx = depth_model.Tx(), depth_Ty = depth_model.Ty();
  double rgb_fx = rgb_model.fx(), rgb_fy = rgb_model.fy();
  double rgb_cx = rgb_model.cx(), rgb_cy = rgb_model.cy();
  double rgb_Tx = rgb_model.Tx(), rgb_Ty = rgb_model.Ty();

  // Transform the depth values into the RGB frame
  /// @todo When RGB is higher res, interpolate by rasterizing depth triangles onto the registered image
  const T *depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  T *registered_data = reinterpret_cast<T *>(&registered_msg->data[0]);
  int raw_index = 0;
  for (unsigned v = 0; v < depth_msg->height; ++v, depth_row += row_step)
  {
    for (unsigned u = 0; u < depth_msg->width; ++u, ++raw_index)
    {
      T raw_depth = depth_row[u];
      if (!DepthTraits<T>::valid(raw_depth))
        continue;

      double depth = DepthTraits<T>::toMeters(raw_depth);

      /// @todo Combine all operations into one matrix multiply on (u,v,d)
      // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
      Eigen::Vector4d xyz_depth;
      xyz_depth << ((u - depth_cx) * depth - depth_Tx) * inv_depth_fx,
          ((v - depth_cy) * depth - depth_Ty) * inv_depth_fy,
          depth,
          1;

      // Transform to RGB camera frame
      Eigen::Vector4d xyz_rgb = depth_to_rgb * xyz_depth;

      // Project to (u,v) in RGB image
      double inv_Z = 1.0 / xyz_rgb.z();
      int u_rgb = (rgb_fx * xyz_rgb.x() + rgb_Tx) * inv_Z + rgb_cx + 0.5;
      int v_rgb = (rgb_fy * xyz_rgb.y() + rgb_Ty) * inv_Z + rgb_cy + 0.5;

      if (u_rgb < 0 || u_rgb >= (int)registered_msg->width ||
          v_rgb < 0 || v_rgb >= (int)registered_msg->height)
        continue;

      T &reg_depth = registered_data[v_rgb * registered_msg->width + u_rgb];
      T new_depth = DepthTraits<T>::fromMeters(xyz_rgb.z());
      // Validity and Z-buffer checks
      if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth)
        reg_depth = new_depth;
    }
  }
}

sensor_msgs::ImagePtr imageCb(const sensor_msgs::ImageConstPtr &depth_image_msg,
                              const sensor_msgs::CameraInfoConstPtr &depth_info_msg,
                              const sensor_msgs::CameraInfoConstPtr &rgb_info_msg,
                              const std::string &fixed_frame,
                              const tf2_ros::Buffer &tf_buffer)
{
  image_geometry::PinholeCameraModel depth_model, rgb_model;
  depth_model.fromCameraInfo(depth_info_msg);
  rgb_model.fromCameraInfo(rgb_info_msg);

  // Query tf2 for transform from (X,Y,Z) in depth camera frame to RGB camera frame
  Eigen::Affine3d depth_to_rgb;
  try
  {
    // Try to account for motion (requires transform to fixed frame)
    geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
        rgb_info_msg->header.frame_id,
        rgb_info_msg->header.stamp,
        depth_info_msg->header.frame_id,
        depth_info_msg->header.stamp,
        fixed_frame);

    tf::transformMsgToEigen(transform.transform, depth_to_rgb);
  }
  catch (tf2::TransformException &ex)
  {
    // If that didn't work, try without accounting for motion
    geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
        rgb_info_msg->header.frame_id,
        depth_info_msg->header.frame_id,
        depth_info_msg->header.stamp);

    tf::transformMsgToEigen(transform.transform, depth_to_rgb);
  }

  // Allocate registered depth image
  sensor_msgs::ImagePtr registered_msg(new sensor_msgs::Image);
  registered_msg->header.stamp = depth_image_msg->header.stamp;
  registered_msg->header.frame_id = rgb_info_msg->header.frame_id;
  registered_msg->encoding = depth_image_msg->encoding;

  cv::Size resolution = rgb_model.reducedResolution();
  registered_msg->height = resolution.height;
  registered_msg->width = resolution.width;
  // step and data set in convert(), depend on depth data type

  if (depth_image_msg->encoding == enc::TYPE_16UC1)
  {
    convert<uint16_t>(depth_image_msg, registered_msg, depth_to_rgb, depth_model, rgb_model);
  }
  else if (depth_image_msg->encoding == enc::TYPE_32FC1)
  {
    convert<float>(depth_image_msg, registered_msg, depth_to_rgb, depth_model, rgb_model);
  }
  else
  {
    printf("Depth image has unsupported encoding [%s]", depth_image_msg->encoding.c_str());
    exit(1);
  }

  return registered_msg;
}

} // end namespace depth_image_proc