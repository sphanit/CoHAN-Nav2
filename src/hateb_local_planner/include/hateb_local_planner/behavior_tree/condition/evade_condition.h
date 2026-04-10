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

#include <rclcpp/rclcpp.hpp>
// #include <rclcpp_lifecycle/lifecycle_node.hpp>

// New
#include <hateb_local_planner/behavior_tree/bt_core.h>

#include <agent_path_prediction/msg/agents_info.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <hateb_local_planner/agents_class.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hateb_local_planner {

/**
 * @brief Class implementing a condition node for checking evade criteria
 *
 * This class provides functionality to determine if an evasion needs to be tiggerred, which then could be execute evade action.
 * It inherits from BT::ConditionNode to integrate with the behavior tree framework.
 */
class EvadeCondition : public BT::ConditionNode {
 public:
  /**
   * @brief Constructor with condition name and configuration
   * @param condition_name Name of the condition node
   * @param conf Configuration for the behavior tree node
   */
  EvadeCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

  /**
   * @brief Deleted default constructor to enforce proper initialization
   */
  EvadeCondition() = delete;

  /**
   * @brief Virtual destructor for cleanup
   */
  ~EvadeCondition() override;

  /**
   * @brief Method called to evaluate the conditione dist
   * @return Status indicating whether condition is met
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input ports for condition evaluation
   * @return Ports list containing agents_info and dist_max as inputs
   */
  static BT::PortsList providedPorts() { return {BT::InputPort<agent_path_prediction::msg::AgentsInfo>("agents_info"), BT::InputPort<geometry_msgs::msg::Pose>("nearest_corner")}; }

 private:
  /**
   * @brief Publishes visualization markers for the evasion vectors
   */
  // void publishVectors();
  // Blackboard entries
  agent_path_prediction::msg::AgentsInfo agents_info_;  //!< Information about agents in the environment
  geometry_msgs::msg::Pose nearest_corner_;             //!< Position of the nearest corner
  double mid_x_;                                        //!< Midpoint x-coordinate between human and corner
  double mid_y_;                                        //!< Midpoint y-coordinate between human and corner
  std::pair<double, double> r_dx_dy_;                   //!< Pair to store the evasion direction vector (dx, dy)
  bool evasion_triggered_;                              //!< Flag to indicate if evasion has been triggered

  // // ROS
  // rclcpp_lifecycle::LifecycleNode::SharedPtr node_;                                //!< ROS node for publishing
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vector_pub_;  //!< Publisher for vector markers

  // name of the node
  std::string name_;  //!< Name of the condition node
};
};  // namespace hateb_local_planner
