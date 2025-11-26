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
#ifndef HATEB_GOAL_CHECKER_H_
#define HATEB_GOAL_CHECKER_H_

#include <memory>
#include <nav2_core/goal_checker.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vector>

namespace hateb_local_planner {
class HATEBGoalChecker : public nav2_core::GoalChecker {
 public:
  /**
   * @brief Constructor
   */
  HATEBGoalChecker();

  /**
   * @brief Destructor
   */
  ~HATEBGoalChecker() = default;

  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(const geometry_msgs::msg::Pose& query_pose, const geometry_msgs::msg::Pose& goal_pose, const geometry_msgs::msg::Twist& velocity) override;

  bool getTolerances(geometry_msgs::msg::Pose& pose_tolerance, geometry_msgs::msg::Twist& vel_tolerance) override;

  void setGoalControl(bool enable) { goal_ctrl_ = enable; }

 protected:
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_;
  bool goal_ctrl_;

  // Cached squared xy_goal_tolerance_
  double xy_goal_tolerance_sq_;
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};

}  // namespace hateb_local_planner

#endif  // HATEB_GOAL_CHECKER_H_