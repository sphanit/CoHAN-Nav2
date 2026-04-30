#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <evasion_planner/evasion_planner.hpp>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace evasion_planner {

void EvasionPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  navfn_.configure(node_, name_ + ".navfn", tf, costmap_ros);

  // Subscribe to planning mode and evasion control point topics
  planning_mode_sub_ = node_->create_subscription<hateb_local_planner::msg::PlanningMode>("planning_mode", 10, std::bind(&EvasionPlanner::planningModeCallback, this, std::placeholders::_1));
  evasion_control_point_sub_ = node_->create_subscription<geometry_msgs::msg::Point>("evasion_control_point", 10, std::bind(&EvasionPlanner::evasionControlPointCallback, this, std::placeholders::_1));

  // Initialize last goal to an invalid position
  last_goal_.x = 999.0;
  last_goal_.y = 999.0;
}

void EvasionPlanner::cleanup() { navfn_.cleanup(); }
void EvasionPlanner::activate() { navfn_.activate(); }
void EvasionPlanner::deactivate() { navfn_.deactivate(); }

nav_msgs::msg::Path EvasionPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) {
  if ((last_goal_ - Point(goal.pose.position.x, goal.pose.position.y)).norm() > 0.1) {
    current_planning_mode_.plan_mode = 0;  // Reset to default planning mode
    last_path_.poses.clear();
    last_goal_ = Point(goal.pose.position.x, goal.pose.position.y);
  }

  geometry_msgs::msg::PoseStamped control_point_pose;
  control_point_pose.header.frame_id = "map";
  control_point_pose.header.stamp = node_->get_clock()->now();
  control_point_pose.pose.position.x = evasion_control_point_.x;
  control_point_pose.pose.position.y = evasion_control_point_.y;
  control_point_pose.pose.position.z = 0.0;
  control_point_pose.pose.orientation.w = 1.0;

  if (static_cast<int>(current_planning_mode_.plan_mode) == 4) {
    if (!last_path_.poses.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Using cached path");
      // Publish cached paths again
      return last_path_;
    }
    try {
      auto path1 = navfn_.createPlan(start, control_point_pose);
      auto path2 = navfn_.createPlan(control_point_pose, goal);

      if (!path1.poses.empty() && !path2.poses.empty()) {
        last_path_ = combinePaths(path1, path2);
        return last_path_;
      }
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "Failed to get a combined path");
    }
  }

  return navfn_.createPlan(start, goal);
}

nav_msgs::msg::Path EvasionPlanner::combinePaths(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2) {
  nav_msgs::msg::Path combined = path1;

  // Remove last pose if same as first of path2
  if (!combined.poses.empty() && !path2.poses.empty()) {
    auto& last = combined.poses.back().pose.position;
    auto& first = path2.poses.front().pose.position;
    if (hypot(last.x - first.x, last.y - first.y) < 1e-6) {  // same point
      combined.poses.pop_back();
    }
  }

  combined.poses.insert(combined.poses.end(), path2.poses.begin(), path2.poses.end());
  return combined;
}

}  // namespace evasion_planner

PLUGINLIB_EXPORT_CLASS(evasion_planner::EvasionPlanner, nav2_core::GlobalPlanner)