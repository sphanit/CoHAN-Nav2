/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2026 INRIA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Author: Phani Teja Singamaneni
 *********************************************************************************/

#include <hateb_local_planner/behavior_tree/condition/evade_condition.h>

namespace hateb_local_planner {

EvadeCondition::EvadeCondition(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf) {
  // Initialize the node
  name_ = condition_name;
  mid_x_ = std::numeric_limits<double>::max();
  mid_y_ = std::numeric_limits<double>::max();
  evasion_triggered_ = false;
  // node_ = nullptr;
  // vector_pub_ = nullptr;
  BT_INFO(name_, "Starting the EvadeCondition BT Node");
}

EvadeCondition::~EvadeCondition() {
  // BT_INFO in destructor
  BT_INFO(name_, "Shutting down the EvadeCondition BT Node");
}

BT::NodeStatus EvadeCondition::tick() {
  // Initialize publisher if not done
  if (!node_) {
    node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
    vector_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/evade_vectors", 1);
  }

  // Read the values from blackboard
  getInput("agents_info", agents_info_);
  getInput("nearest_corner", nearest_corner_);
  getInput("recovery", evasion_triggered_);
  getInput("look_ahead_goal", goal_);

  if (!evasion_triggered_) {
    // Check if there are any humans to evade
    if (agents_info_.humans.empty()) {
      BT_INFO(name_, "No humans detected, evasion not needed.")
      publishVectors();
      return BT::NodeStatus::FAILURE;
    }

    // Check if Evasion is needed or not
    if (!doSegmentsIntersect(Point(agents_info_.robot_pose.x, agents_info_.robot_pose.y), Point(goal_.pose.position.x, goal_.pose.position.y),
                             Point(agents_info_.humans[0].pose.x, agents_info_.humans[0].pose.y), Point(nearest_corner_.position.x, nearest_corner_.position.y))) {
      BT_INFO(name_, "Evasion not needed! Human is not on the path to the goal.")
      publishVectors();  // Publish even when not evading
      return BT::NodeStatus::FAILURE;
    }

    // if (mid_x_ == std::numeric_limits<double>::max() || mid_y_ == std::numeric_limits<double>::max()) {
    mid_x_ = (nearest_corner_.position.x + agents_info_.humans[0].pose.x) / 2.;
    mid_y_ = (nearest_corner_.position.y + agents_info_.humans[0].pose.y) / 2.;
    r_dx_dy_ = std::make_pair(mid_x_ - agents_info_.robot_pose.x, mid_y_ - agents_info_.robot_pose.y);
    // }

    auto dx = nearest_corner_.position.x - agents_info_.humans[0].pose.x;
    auto dy = nearest_corner_.position.y - agents_info_.humans[0].pose.y;

    if (std::hypot(dx, dy) > 1.8) {  // Safety distance threshold for evasion, can be made configurable
      BT_INFO(name_, "Evasion not needed! Human is far from the corner.")
      publishVectors();  // Publish even when not evading
      return BT::NodeStatus::FAILURE;
    }
  }

  auto r_dx = mid_x_ - agents_info_.robot_pose.x;
  auto r_dy = mid_y_ - agents_info_.robot_pose.y;
  if (r_dx_dy_.first * r_dx + r_dx_dy_.second * r_dy <= 0) {
    evasion_triggered_ = false;
    setOutput("recovery", false);
    BT_INFO(name_, "Evasion completed!")
    publishVectors();
    return BT::NodeStatus::FAILURE;
  } else if (r_dx_dy_.first * r_dx + r_dx_dy_.second * r_dy > 0) {
    evasion_triggered_ = true;
    setOutput("recovery", true);
  }

  BT_INFO(name_, "Evasion in progress...")
  publishVectors();
  return BT::NodeStatus::SUCCESS;
}

void EvadeCondition::publishVectors() {
  visualization_msgs::msg::MarkerArray marker_array;

  // Marker for initial vector r_dx_dy_
  visualization_msgs::msg::Marker initial_vector;
  initial_vector.header.frame_id = "map";  // Assuming map frame
  initial_vector.header.stamp = node_->now();
  initial_vector.ns = "evade_vectors";
  initial_vector.id = 0;
  initial_vector.type = visualization_msgs::msg::Marker::ARROW;
  initial_vector.action = visualization_msgs::msg::Marker::ADD;
  initial_vector.pose.position.x = agents_info_.robot_pose.x;
  initial_vector.pose.position.y = agents_info_.robot_pose.y;
  initial_vector.pose.position.z = 0.0;
  initial_vector.pose.orientation.w = 1.0;
  initial_vector.scale.x = 0.1;  // shaft diameter
  initial_vector.scale.y = 0.2;  // head diameter
  initial_vector.scale.z = 0.2;  // head length
  initial_vector.color.r = 1.0;
  initial_vector.color.g = 0.0;
  initial_vector.color.b = 0.0;
  initial_vector.color.a = 1.0;
  // Points: start and end
  geometry_msgs::msg::Point start;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;
  geometry_msgs::msg::Point end;
  end.x = r_dx_dy_.first;
  end.y = r_dx_dy_.second;
  end.z = 0.0;
  initial_vector.points.push_back(start);
  initial_vector.points.push_back(end);

  // Marker for current vector (r_dx, r_dy)
  visualization_msgs::msg::Marker current_vector;
  current_vector.header.frame_id = "map";
  current_vector.header.stamp = node_->now();
  current_vector.ns = "evade_vectors";
  current_vector.id = 1;
  current_vector.type = visualization_msgs::msg::Marker::ARROW;
  current_vector.action = visualization_msgs::msg::Marker::ADD;
  current_vector.pose.position.x = agents_info_.robot_pose.x;
  current_vector.pose.position.y = agents_info_.robot_pose.y;
  current_vector.pose.position.z = 0.0;
  current_vector.pose.orientation.w = 1.0;
  current_vector.scale.x = 0.1;
  current_vector.scale.y = 0.2;
  current_vector.scale.z = 0.2;
  current_vector.color.r = 0.0;
  current_vector.color.g = 1.0;
  current_vector.color.b = 0.0;
  current_vector.color.a = 1.0;
  // Points
  auto r_dx = mid_x_ - agents_info_.robot_pose.x;
  auto r_dy = mid_y_ - agents_info_.robot_pose.y;
  geometry_msgs::msg::Point end_curr;
  end_curr.x = r_dx;
  end_curr.y = r_dy;
  end_curr.z = 0.0;
  current_vector.points.push_back(start);
  current_vector.points.push_back(end_curr);

  marker_array.markers.push_back(initial_vector);
  marker_array.markers.push_back(current_vector);

  vector_pub_->publish(marker_array);
}
}  // namespace hateb_local_planner