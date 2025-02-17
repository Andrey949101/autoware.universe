// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include "autoware/universe_utils/math/trigonometry.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

namespace pointcloud_preprocessor
{
template <class T>
void DistortionCorrector<T>::processTwistMessage(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header = twist_msg->header;
  msg.twist = twist_msg->twist.twist;
  twist_queue_.push_back(msg);

  while (!twist_queue_.empty()) {
    // for replay rosbag
    if (rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(twist_msg->header.stamp)) {
      twist_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(twist_queue_.front().header.stamp) <
      rclcpp::Time(twist_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_queue_.pop_front();
    } else {
      break;
    }
  }
}

template <class T>
void DistortionCorrector<T>::processIMUMessage(
  const std::string & base_frame, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr =
    std::make_shared<geometry_msgs::msg::TransformStamped>();
  getIMUTransformation(base_frame, imu_msg->header.frame_id, geometry_imu_to_base_link_ptr);
  enqueueIMU(imu_msg, geometry_imu_to_base_link_ptr);
}

template <class T>
void DistortionCorrector<T>::getIMUTransformation(
  const std::string & base_frame, const std::string & imu_frame,
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr)
{
  if (imu_transform_exists_) {
    return;
  }

  tf2::Transform tf2_imu_to_base_link;
  if (base_frame == imu_frame) {
    tf2_imu_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_imu_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    imu_transform_exists_ = true;
  } else {
    try {
      const auto transform_msg =
        tf_buffer_.lookupTransform(base_frame, imu_frame, tf2::TimePointZero);
      tf2::convert(transform_msg.transform, tf2_imu_to_base_link);
      imu_transform_exists_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      RCLCPP_ERROR(
        node_->get_logger(), "Please publish TF %s to %s", base_frame.c_str(), imu_frame.c_str());

      tf2_imu_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2_imu_to_base_link.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    }
  }

  geometry_imu_to_base_link_ptr->transform.rotation =
    tf2::toMsg(tf2_imu_to_base_link.getRotation());
}

template <class T>
void DistortionCorrector<T>::enqueueIMU(
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr)
{
  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.vector = imu_msg->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *geometry_imu_to_base_link_ptr);
  transformed_angular_velocity.header = imu_msg->header;
  angular_velocity_queue_.push_back(transformed_angular_velocity);

  while (!angular_velocity_queue_.empty()) {
    // for replay rosbag
    if (
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) >
      rclcpp::Time(imu_msg->header.stamp)) {
      angular_velocity_queue_.pop_front();
    } else if (  // NOLINT
      rclcpp::Time(angular_velocity_queue_.front().header.stamp) <
      rclcpp::Time(imu_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      angular_velocity_queue_.pop_front();
    } else {
      break;
    }
  }
}

template <class T>
void DistortionCorrector<T>::getTwistAndIMUIterator(
  bool use_imu, double first_point_time_stamp_sec,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu)
{
  it_twist = std::lower_bound(
    std::begin(twist_queue_), std::end(twist_queue_), first_point_time_stamp_sec,
    [](const geometry_msgs::msg::TwistStamped & x, const double t) {
      return rclcpp::Time(x.header.stamp).seconds() < t;
    });
  it_twist = it_twist == std::end(twist_queue_) ? std::end(twist_queue_) - 1 : it_twist;

  if (use_imu && !angular_velocity_queue_.empty()) {
    it_imu = std::lower_bound(
      std::begin(angular_velocity_queue_), std::end(angular_velocity_queue_),
      first_point_time_stamp_sec, [](const geometry_msgs::msg::Vector3Stamped & x, const double t) {
        return rclcpp::Time(x.header.stamp).seconds() < t;
      });
    it_imu =
      it_imu == std::end(angular_velocity_queue_) ? std::end(angular_velocity_queue_) - 1 : it_imu;
  }
}

template <class T>
bool DistortionCorrector<T>::isInputValid(sensor_msgs::msg::PointCloud2 & pointcloud)
{
  if (pointcloud.data.empty() || twist_queue_.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000 /* ms */,
      "input pointcloud or twist_queue_ is empty.");
    return false;
  }

  auto time_stamp_field_it = std::find_if(
    std::cbegin(pointcloud.fields), std::cend(pointcloud.fields),
    [](const sensor_msgs::msg::PointField & field) { return field.name == "time_stamp"; });
  if (time_stamp_field_it == pointcloud.fields.cend()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000 /* ms */,
      "Required field time stamp doesn't exist in the point cloud.");
    return false;
  }
  return true;
}

template <class T>
void DistortionCorrector<T>::undistortPointCloud(
  bool use_imu, sensor_msgs::msg::PointCloud2 & pointcloud)
{
  if (!isInputValid(pointcloud)) return;

  sensor_msgs::PointCloud2Iterator<float> it_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(pointcloud, "z");
  sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(pointcloud, "time_stamp");

  double prev_time_stamp_sec{*it_time_stamp};
  const double first_point_time_stamp_sec{*it_time_stamp};

  std::deque<geometry_msgs::msg::TwistStamped>::iterator it_twist;
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator it_imu;
  getTwistAndIMUIterator(use_imu, first_point_time_stamp_sec, it_twist, it_imu);

  // For performance, do not instantiate `rclcpp::Time` inside of the for-loop
  double twist_stamp = rclcpp::Time(it_twist->header.stamp).seconds();
  double imu_stamp{0.0};
  if (use_imu && !angular_velocity_queue_.empty()) {
    imu_stamp = rclcpp::Time(it_imu->header.stamp).seconds();
  }

  // If there is a point in a pointcloud that cannot be associated, record it to issue a warning
  bool is_twist_time_stamp_too_late = false;
  bool is_imu_time_stamp_too_late = false;
  bool is_twist_valid = true;
  bool is_imu_valid = true;

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp) {
    is_twist_valid = true;
    is_imu_valid = true;

    // Get closest twist information
    while (it_twist != std::end(twist_queue_) - 1 && *it_time_stamp > twist_stamp) {
      ++it_twist;
      twist_stamp = rclcpp::Time(it_twist->header.stamp).seconds();
    }
    if (std::abs(*it_time_stamp - twist_stamp) > 0.1) {
      is_twist_time_stamp_too_late = true;
      is_twist_valid = false;
    }

    // Get closest IMU information
    if (use_imu && !angular_velocity_queue_.empty()) {
      while (it_imu != std::end(angular_velocity_queue_) - 1 && *it_time_stamp > imu_stamp) {
        ++it_imu;
        imu_stamp = rclcpp::Time(it_imu->header.stamp).seconds();
      }

      if (std::abs(*it_time_stamp - imu_stamp) > 0.1) {
        is_imu_time_stamp_too_late = true;
        is_imu_valid = false;
      }
    } else {
      is_imu_valid = false;
    }

    float time_offset = static_cast<float>(*it_time_stamp - prev_time_stamp_sec);

    // Undistort a single point based on the strategy
    undistortPoint(it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);

    prev_time_stamp_sec = *it_time_stamp;
  }

  warnIfTimestampIsTooLate(is_twist_time_stamp_too_late, is_imu_time_stamp_too_late);
}

template <class T>
void DistortionCorrector<T>::warnIfTimestampIsTooLate(
  bool is_twist_time_stamp_too_late, bool is_imu_time_stamp_too_late)
{
  if (is_twist_time_stamp_too_late) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000 /* ms */,
      "Twist time_stamp is too late. Could not interpolate.");
  }

  if (is_imu_time_stamp_too_late) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000 /* ms */,
      "IMU time_stamp is too late. Could not interpolate.");
  }
}

///////////////////////// Functions for different undistortion strategies /////////////////////////

void DistortionCorrector2D::initialize()
{
  x_ = 0.0f;
  y_ = 0.0f;
  theta_ = 0.0f;
}

void DistortionCorrector3D::initialize()
{
  prev_transformation_matrix_ = Eigen::Matrix4f::Identity();
}

void DistortionCorrector2D::setPointCloudTransform(
  const std::string & base_frame, const std::string & lidar_frame)
{
  if (pointcloud_transform_exists_) {
    return;
  }

  if (base_frame == lidar_frame) {
    tf2_lidar_to_base_link_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2_lidar_to_base_link_.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    tf2_base_link_to_lidar_ = tf2_lidar_to_base_link_;
    pointcloud_transform_exists_ = true;
  } else {
    try {
      const auto transform_msg =
        tf_buffer_.lookupTransform(base_frame, lidar_frame, tf2::TimePointZero);
      tf2::convert(transform_msg.transform, tf2_lidar_to_base_link_);
      tf2_base_link_to_lidar_ = tf2_lidar_to_base_link_.inverse();
      pointcloud_transform_exists_ = true;
      pointcloud_transform_needed_ = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      RCLCPP_ERROR(
        node_->get_logger(), "Please publish TF %s to %s", base_frame.c_str(), lidar_frame.c_str());

      tf2_lidar_to_base_link_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2_lidar_to_base_link_.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      tf2_base_link_to_lidar_ = tf2_lidar_to_base_link_;
    }
  }
}

void DistortionCorrector3D::setPointCloudTransform(
  const std::string & base_frame, const std::string & lidar_frame)
{
  if (pointcloud_transform_exists_) {
    return;
  }

  if (base_frame == lidar_frame) {
    eigen_lidar_to_base_link_ = Eigen::Matrix4f::Identity();
    eigen_base_link_to_lidar_ = Eigen::Matrix4f::Identity();
    pointcloud_transform_exists_ = true;
  }

  try {
    const auto transform_msg =
      tf_buffer_.lookupTransform(base_frame, lidar_frame, tf2::TimePointZero);
    eigen_lidar_to_base_link_ =
      tf2::transformToEigen(transform_msg.transform).matrix().cast<float>();
    eigen_base_link_to_lidar_ = eigen_lidar_to_base_link_.inverse();
    pointcloud_transform_exists_ = true;
    pointcloud_transform_needed_ = true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      node_->get_logger(), "Please publish TF %s to %s", base_frame.c_str(), lidar_frame.c_str());
    eigen_lidar_to_base_link_ = Eigen::Matrix4f::Identity();
    eigen_base_link_to_lidar_ = Eigen::Matrix4f::Identity();
  }
}

inline void DistortionCorrector2D::undistortPointImplementation(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
  const bool & is_twist_valid, const bool & is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v{0.0f}, w{0.0f};
  if (is_twist_valid) {
    v = static_cast<float>(it_twist->twist.linear.x);
    w = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid) {
    w = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_tf_.setValue(*it_x, *it_y, *it_z);

  if (pointcloud_transform_needed_) {
    point_tf_ = tf2_lidar_to_base_link_ * point_tf_;
  }
  theta_ += w * time_offset;
  baselink_quat_.setValue(
    0, 0, autoware::universe_utils::sin(theta_ * 0.5f),
    autoware::universe_utils::cos(theta_ * 0.5f));  // baselink_quat.setRPY(0.0, 0.0, theta);
  const float dis = v * time_offset;
  x_ += dis * autoware::universe_utils::cos(theta_);
  y_ += dis * autoware::universe_utils::sin(theta_);

  baselink_tf_odom_.setOrigin(tf2::Vector3(x_, y_, 0.0));
  baselink_tf_odom_.setRotation(baselink_quat_);

  undistorted_point_tf_ = baselink_tf_odom_ * point_tf_;

  if (pointcloud_transform_needed_) {
    undistorted_point_tf_ = tf2_base_link_to_lidar_ * undistorted_point_tf_;
  }

  *it_x = static_cast<float>(undistorted_point_tf_.getX());
  *it_y = static_cast<float>(undistorted_point_tf_.getY());
  *it_z = static_cast<float>(undistorted_point_tf_.getZ());
}

inline void DistortionCorrector3D::undistortPointImplementation(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
  const bool & is_twist_valid, const bool & is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v_x_{0.0f}, v_y_{0.0f}, v_z_{0.0f}, w_x_{0.0f}, w_y_{0.0f}, w_z_{0.0f};
  if (is_twist_valid) {
    v_x_ = static_cast<float>(it_twist->twist.linear.x);
    v_y_ = static_cast<float>(it_twist->twist.linear.y);
    v_z_ = static_cast<float>(it_twist->twist.linear.z);
    w_x_ = static_cast<float>(it_twist->twist.angular.x);
    w_y_ = static_cast<float>(it_twist->twist.angular.y);
    w_z_ = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid) {
    w_x_ = static_cast<float>(it_imu->vector.x);
    w_y_ = static_cast<float>(it_imu->vector.y);
    w_z_ = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_eigen_ << *it_x, *it_y, *it_z, 1.0;
  if (pointcloud_transform_needed_) {
    point_eigen_ = eigen_lidar_to_base_link_ * point_eigen_;
  }

  Sophus::SE3f::Tangent twist(v_x_, v_y_, v_z_, w_x_, w_y_, w_z_);
  twist = twist * time_offset;
  transformation_matrix_ = Sophus::SE3f::exp(twist).matrix();
  transformation_matrix_ = transformation_matrix_ * prev_transformation_matrix_;
  undistorted_point_eigen_ = transformation_matrix_ * point_eigen_;

  if (pointcloud_transform_needed_) {
    undistorted_point_eigen_ = eigen_base_link_to_lidar_ * undistorted_point_eigen_;
  }
  *it_x = undistorted_point_eigen_[0];
  *it_y = undistorted_point_eigen_[1];
  *it_z = undistorted_point_eigen_[2];

  prev_transformation_matrix_ = transformation_matrix_;
}

}  // namespace pointcloud_preprocessor
