/*******************************************************************************
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024-2025 LAAS-CNRS
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

#ifndef MODE_SWITCH_HH_
#define MODE_SWITCH_HH_

#include <hateb_local_planner/agents_class.hpp>
#include <rclcpp/rclcpp.hpp>

// Use system BehaviorTree.CPP v3 (ROS 2 Humble)
#include "behaviortree_cpp_v3/bt_factory.h"

// Messages
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <agent_path_prediction/msg/agents_info.hpp>
#include <cohan_msgs/msg/passage_type.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hateb_local_planner/msg/planning_mode.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

// BT Nodes
#include <hateb_local_planner/behavior_tree/action/set_mode.h>
#include <hateb_local_planner/behavior_tree/bt_core.h>
#include <hateb_local_planner/behavior_tree/condition/dual_band_exit_condition.h>
#include <hateb_local_planner/behavior_tree/condition/evade_condition.h>
#include <hateb_local_planner/behavior_tree/condition/is_goal_updated.h>
#include <hateb_local_planner/behavior_tree/condition/passthrough_condition.h>
#include <hateb_local_planner/behavior_tree/condition/single_band_exit_condition.h>
#include <hateb_local_planner/behavior_tree/condition/vel_obs_exit_condition.h>

// Stdlib
#include <mutex>

// All topics are good here
// #define AGENTS_INFO_SUB "/agents_info"
// #define PLAN_SUB "/plan"
// #define RESULT_SUB "/navigate_to_pose/_action/status"
// #define PASSAGE_SUB "/invisible_humans_detection/passage"
// #define SCAN_SUB "/scan"
// #define MAP_FRAME "map"
// #define FOOTPRINT_FRAME "base_footprint"

namespace hateb_local_planner {

/**
 * @brief Class managing planning mode switches in the HATEB local planner
 *
 * ModeSwitch implements a behavior tree-based decision system for switching
 * between different planning modes (single band, dual band, velocity obstacles,
 * etc.) based on the current situation. It processes information about nearby
 * agents, goals, and passage types to determine the most appropriate planning
 * strategy.
 */
class ModeSwitch {
 public:
  /**
   * @brief Default constructor
   */
  ModeSwitch();

  /**
   * @brief Default destructor
   */
  ~ModeSwitch() = default;

  /**
   * @brief Initializes the mode switch with required components
   * @param node ROS 2 node shared pointer
   * @param cfg Configuration pointer
   * @param agents_ptr Pointer to the agents management class
   * @param tf TF buffer shared pointer
   */
  void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::shared_ptr<HATebConfig> cfg, std::shared_ptr<hateb_local_planner::Agents>& agents_ptr, std::shared_ptr<tf2_ros::Buffer> tf);

  /**
   * @brief Executes one tick of the behavior tree
   * @return Status of the behavior tree after the tick
   */
  BT::NodeStatus tickBT();

  /**
   * @brief Resets the behavior tree to its initial state
   */
  void resetBT();

  /**
   * @brief Gets a pointer to the behavior tree
   * @return Pointer to the behavior tree instance
   */
  BT::Tree* BTree() { return &bhv_tree_; }

  /**
   * @brief Ticks the behavior tree and returns the resulting planning mode
   * @return The selected planning mode after the tree evaluation
   */
  hateb_local_planner::msg::PlanningMode tickAndGetMode();

  /**
   * @brief Sets the behavior tree debug logging flag
   * @param enabled True to enable debug logging, false to disable
   */
  static void setBTDebugEnabled(bool enabled);

 private:
  /**
   * @brief Registers custom nodes with the behavior tree factory
   */
  void registerNodes();

  /**
   * @brief Callback for processing agent information updates
   * @param info_msg Message containing updated agent information
   */
  void agentsInfoCB(const agent_path_prediction::msg::AgentsInfo::SharedPtr info_msg);

  /**
   * @brief Callback for processing new navigation goals
   * @param goal_msg Message containing the new goal
   */
  void planCB(const nav_msgs::msg::Path::SharedPtr plan_msg);

  /**
   * @brief Callback for processing navigation results
   * @param result_msg Message containing the navigation result
   */
  void resultNavigateToPoseCB(const action_msgs::msg::GoalStatusArray::SharedPtr result_msg);

  /**
   * @brief Callback for processing passage type information
   * @param passage_msg Message containing passage classification
   */
  void passageCB(const cohan_msgs::msg::PassageType::SharedPtr passage_msg);

  /**
   * @brief Callback for processing corner information
   * @param scan_msg Message containing laser scan data
   */
  void scanCB(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

  /**
   * @brief Updates the current planning mode
   * @param duration Optional duration parameter for the update
   */
  void updateMode(int duration = 0);

  void printTreeStatus(const BT::TreeNode* node, int level = 0) {
    std::string indent(level * 2, ' ');
    std::cout << indent << node->name() << ": " << toStr(node->status()) << std::endl;

    if (auto control = dynamic_cast<const BT::ControlNode*>(node)) {
      for (unsigned i = 0; i < control->childrenCount(); ++i) {
        printTreeStatus(control->child(i), level + 1);
      }
    }
  }

  void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    latest_scan_.poses.clear();

    // Get the robot's current pose in the map frame
    geometry_msgs::msg::TransformStamped footprint_transform;
    try {
      std::string base_frame = cfg_->footprint_frame;
      if (!ns_.empty()) {
        base_frame = ns_ + "/" + base_frame;
      }
      footprint_transform = tf_->lookupTransform(cfg_->global_frame, base_frame, tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      return;
    }

    tf2::Quaternion q(footprint_transform.transform.rotation.x, footprint_transform.transform.rotation.y, footprint_transform.transform.rotation.z, footprint_transform.transform.rotation.w);
    tf2::Vector3 p(footprint_transform.transform.translation.x, footprint_transform.transform.translation.y, footprint_transform.transform.translation.z);
    tf2::Transform transform(q, p);

    auto ranges = scan_msg->ranges;
    auto angle_increment = scan_msg->angle_increment;
    auto angle_min = scan_msg->angle_min;
    auto samples = scan_msg->ranges.size();
    double ang = angle_min;

    latest_scan_.header.stamp = node_->now();
    latest_scan_.header.frame_id = cfg_->global_frame;

    for (int i = 0; i < samples; i++) {
      double x1 = ranges[i] * cos(ang);
      double y1 = ranges[i] * sin(ang);
      auto laser_point_posistion = tf2::Vector3(x1, y1, 0.);
      laser_point_posistion = transform * laser_point_posistion;
      geometry_msgs::msg::Pose laser_point;
      laser_point.position.x = laser_point_posistion.x();
      laser_point.position.y = laser_point_posistion.y();
      laser_point.orientation.w = 1;
      latest_scan_.poses.push_back(laser_point);
      ang += angle_increment;
    }
  }

  // Status flags
  bool goal_reached_;           //!< Flag indicating if current goal was reached
  bool initialized_;            //!< Flag indicating if class was properly initialized
  bool goal_update_;            //!< Flag indicating if goal was updated
  double look_ahead_distance_;  //!< Distance to look ahead in the plan for evasion decisions

  // ROS communication members
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;                                          //!< ROS 2 node shared pointer
  rclcpp::Subscription<agent_path_prediction::msg::AgentsInfo>::SharedPtr agents_info_sub_;  //!< Subscriber for agent information
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;                            //!< Subscriber for navigation goals
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr result_sub_;            //!< Subscriber for navigation results
  rclcpp::Subscription<cohan_msgs::msg::PassageType>::SharedPtr passage_detect_sub_;         //!< Subscriber for passage detection
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;                    //!< Subscriber for laser scan information

  rclcpp::Publisher<hateb_local_planner::msg::PlanningMode>::SharedPtr planning_mode_pub_;  //!< Publisher for current planning mode
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr evasion_control_point_pub_;       //!< Publisher for evasion control point

  // State information
  geometry_msgs::msg::PoseStamped goal_;                //!< Current navigation goal
  agent_path_prediction::msg::AgentsInfo agents_info_;  //!< Current agent information
  action_msgs::msg::GoalStatusArray result_msg_;        //!< Latest navigation result

  // Behavior Tree components
  BT::BehaviorTreeFactory bhv_factory_;  //!< Factory for creating BT nodes
  BT::Tree bhv_tree_;                    //!< The behavior tree instance

  std::mutex pub_mutex_;   //!< Mutex for thread-safe publishing
  std::mutex scan_mutex_;  //!< Mutex for thread-safe access to latest scan data

  std::string name_;                                  //!< Name of this node
  hateb_local_planner::msg::PlanningMode plan_mode_;  //!< Current planning mode
  ModeInfo mode_info_;                                //!< Detailed mode information

  // Params for namespace and subscription topics
  std::string ns_;                     //!< Namespace of the node
  std::string agents_info_sub_topic_;  //!< Topic for agents information
  std::string plan_sub_topic_;         //!< Topic for robot plan
  std::string result_sub_topic_;       //!< Topic for robot goal result
  std::string passage_sub_topic_;      //!< Topic for passage detection (from invisible humans)
  std::string scan_sub_topic_;         //!< Topic for laser scan data

  // TF buffer for transformations
  std::shared_ptr<tf2_ros::Buffer> tf_;        //!< TF buffer for transformations
  geometry_msgs::msg::PoseArray latest_scan_;  //!< Latest laser scan data
  std::shared_ptr<HATebConfig> cfg_;           //!< Config class that stores and manages all related parameters
};

}  // namespace hateb_local_planner
#endif  // MODE_SWITCH_HH_
