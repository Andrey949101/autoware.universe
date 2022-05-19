// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_CORE_HPP_
#define FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_CORE_HPP_

#include "freespace_planning_algorithms/reeds_shepp.hpp"

#include <boost/optional.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace rrtstar_core
{
using Path = freespace_planning_algorithms::ReedsSheppStateSpace::ReedsSheppPath;
using Pose = freespace_planning_algorithms::ReedsSheppStateSpace::StateXYT;
using ReedsSheppStateSpace = freespace_planning_algorithms::ReedsSheppStateSpace;
const double inf = std::numeric_limits<double>::infinity();

class CSpace
{
public:
  CSpace(const Pose & lo, const Pose & hi, double r, std::function<bool(Pose)> is_obstacle_free);
  Pose uniformSampling();
  Pose ellipticInformedSampling(double c_best, const Pose & p_start, const Pose & p_goal);
  double distance(const Pose & pose0, const Pose & pose1) const
  {
    return rsspace_.distance(pose0, pose1);
  }
  double distanceLowerBound(const Pose & pose0, const Pose & pose1) const
  {
    return std::hypot(pose0.x - pose1.x, pose0.y - pose1.y);
  };

  // Method names with postfix child2parent indicate that the arguments regarding two nodes must
  // follow child-to-parent order.
  //
  // Note that parent-to-child and child-to-parent reeds-shepp paths are sometimes
  // different, because there may two reeds-shepp paths with an exact same cost given two poses.
  // For example, say you want to compute a reeds-shepp path from [0, 0, 0] to [0, 1, 0], where
  // each element indicate x, y, and yaw. In such a case, you will have symetric two reeds-sheep
  // paths, one of which starts moving with forward motion and
  // the other of which with backward motion.
  // So, due to floating point level difference, the resulting reeds-shepp path can be different.
  //
  // In this code, by forcing all reeds-shepp-involved methods take arguments in the
  // child-to-parent order, we keep the consistency of the resulting reeds-shepp paths.
  void sampleWayPoints_child2parent(
    const Pose & pose_child, const Pose & pose_parent, double step,
    std::vector<Pose> & pose_seq) const;
  Pose interpolate_child2parent(
    const Pose & pose_child, const Pose & pose_parent, double seg) const;
  bool isValidPath_child2parent(
    const Pose & pose_child, const Pose & pose_parent, double step) const;

  bool isValidPose(const Pose & pose) const { return isInside(pose) && is_obstacle_free_(pose); }
  double getReedsSheppRadius() const { return rsspace_.rho_; }

private:
  bool isInside(const Pose & p) const;
  Pose interpolate(Path & path, const Pose & pose, double seg) const;
  const Pose lo_;
  const Pose hi_;
  const ReedsSheppStateSpace rsspace_;
  std::function<bool(Pose)> is_obstacle_free_;
  std::mt19937 rand_gen_;
};

class RRTStar
{
public:
  struct Node
  {
    Pose pose;
    double cost_from_start;
    double cost_to_goal;
    boost::optional<size_t> idx = boost::none;
    boost::optional<size_t> parent_idx = boost::none;
    boost::optional<double> cost_to_parent = boost::none;
    std::vector<size_t> child_indexes = std::vector<size_t>();

    void addParent(size_t parent_idx_, double cost_to_parent_)
    {
      parent_idx = parent_idx_;
      cost_to_parent = cost_to_parent_;
    }
  };

  RRTStar(
    Pose x_start, Pose x_goal, double mu, double collision_check_resolution, bool is_informed,
    CSpace cspace);
  bool isSolutionFound() const { return (reached_nodes_.size() > 0); }
  void extend();
  std::vector<Pose> sampleSolutionWaypoints(double resolution) const;
  void dumpState(std::string filename) const;
  double getSolutionCost() const { return node_goal_.cost_from_start; }

private:
  size_t findNearestIndex(const Pose & x_rand) const;
  std::vector<size_t> findNeighboreIndexes(const Pose & pose) const;
  Node & addNewNode(const Pose & pose, size_t idx_parent);
  Node & getParentNode(const Node & node) { return nodes_[*node.parent_idx]; }
  void rewire(Node & node_a, Node & node_b);
  size_t getBestParentIndex(
    const Pose & pose_new, const Node & node_nearest,
    const std::vector<size_t> & neighbore_indexes) const;
  boost::optional<size_t> getRewireTargetIndex(
    const Node & node_new, const std::vector<size_t> & neighbore_indexes) const;

  Node node_start_;
  Node node_goal_;
  std::vector<Node> nodes_;
  std::vector<Node> reached_nodes_;
  const double mu_;
  const double collision_check_resolution_;
  const bool is_informed_;
  CSpace cspace_;
};

}  // namespace rrtstar_core

#endif  // FREESPACE_PLANNING_ALGORITHMS__RRTSTAR_CORE_HPP_
