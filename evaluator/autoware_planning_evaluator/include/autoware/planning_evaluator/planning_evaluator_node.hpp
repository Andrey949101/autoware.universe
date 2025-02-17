// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_

#include "autoware/planning_evaluator/metrics_calculator.hpp"
#include "autoware/planning_evaluator/stat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>

#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace planning_diagnostics
{
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using nav_msgs::msg::Odometry;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::AccelWithCovarianceStamped;
/**
 * @brief Node for planning evaluation
 */
class PlanningEvaluatorNode : public rclcpp::Node
{
public:
  explicit PlanningEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~PlanningEvaluatorNode();

  /**
   * @brief callback on receiving an odometry
   * @param [in] odometry_msg received odometry message
   */
  void onOdometry(const Odometry::ConstSharedPtr odometry_msg);

  /**
   * @brief callback on receiving a trajectory
   * @param [in] traj_msg received trajectory message
   */
  void onTrajectory(const Trajectory::ConstSharedPtr traj_msg);

  /**
   * @brief callback on receiving a reference trajectory
   * @param [in] traj_msg received trajectory message
   */
  void onReferenceTrajectory(const Trajectory::ConstSharedPtr traj_msg);

  /**
   * @brief callback on receiving a dynamic objects array
   * @param [in] objects_msg received dynamic object array message
   */
  void onObjects(const PredictedObjects::ConstSharedPtr objects_msg);

  /**
   * @brief callback on receiving a modified goal
   * @param [in] modified_goal_msg received modified goal message
   */
  void onModifiedGoal(const PoseWithUuidStamped::ConstSharedPtr modified_goal_msg);

  /**
   * @brief publish the given metric statistic
   */
  DiagnosticStatus generateDiagnosticStatus(
    const Metric & metric, const Stat<double> & metric_stat) const;

  /**
   * @brief publish current ego lane info
   */
  DiagnosticStatus generateLaneletDiagnosticStatus();

  /**
   * @brief publish current ego kinematic state
   */
  DiagnosticStatus generateKinematicStateDiagnosticStatus(
    const AccelWithCovarianceStamped & accel_stamped);

private:
  static bool isFinite(const TrajectoryPoint & p);
  void publishModifiedGoalDeviationMetrics();
  // update Route Handler
  void getRouteData();

  // ROS
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr ref_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr modified_goal_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  autoware::universe_utils::InterProcessPollingSubscriber<LaneletRoute> route_subscriber_{
    this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<LaneletMapBin> vector_map_subscriber_{
    this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> accel_sub_{
    this, "~/input/acceleration"};

  rclcpp::Publisher<DiagnosticArray>::SharedPtr metrics_pub_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  autoware::route_handler::RouteHandler route_handler_;

  // Parameters
  std::string output_file_str_;
  std::string ego_frame_str_;

  // Calculator
  MetricsCalculator metrics_calculator_;
  // Metrics
  std::vector<Metric> metrics_;
  std::deque<rclcpp::Time> stamps_;
  std::array<std::deque<Stat<double>>, static_cast<size_t>(Metric::SIZE)> metric_stats_;

  Odometry::ConstSharedPtr ego_state_ptr_;
  PoseWithUuidStamped::ConstSharedPtr modified_goal_ptr_;
  std::optional<AccelWithCovarianceStamped> prev_acc_stamped_{std::nullopt};
};
}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_
