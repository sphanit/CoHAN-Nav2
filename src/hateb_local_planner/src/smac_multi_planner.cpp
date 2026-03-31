#include <tf2/utils.h>

#include <hateb_local_planner/smac_multi_planner.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace smac_multi_planner {

void SmacMultiPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;

  node_->declare_parameter(name_ + ".num_paths", 5);
  node_->declare_parameter(name_ + ".goal_yaw_offset", 0.3);
  node_->declare_parameter(name_ + ".goal_xy_offset", 1.0);
  node_->declare_parameter(name_ + ".mid_yaw_offset", 0.1);
  node_->declare_parameter(name_ + ".mid_xy_offset", 0.3);

  node_->get_parameter(name_ + ".num_paths", num_paths_);
  node_->get_parameter(name_ + ".goal_yaw_offset", goal_yaw_offset_);
  node_->get_parameter(name_ + ".goal_xy_offset", goal_xy_offset_);
  node_->get_parameter(name_ + ".mid_yaw_offset", mid_yaw_offset_);
  node_->get_parameter(name_ + ".mid_xy_offset", mid_xy_offset_);

  costmap_ros_ = costmap_ros;

  // Configure internal Navfn planner
  navfn_.configure(node_, name_ + ".navfn", tf, costmap_ros);

  debug_pub_ = node_->create_publisher<nav_msgs::msg::Path>("smac_multi_paths", 10);
}

void SmacMultiPlanner::cleanup() { navfn_.cleanup(); }
void SmacMultiPlanner::activate() { navfn_.activate(); }
void SmacMultiPlanner::deactivate() { navfn_.deactivate(); }

nav_msgs::msg::Path SmacMultiPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) {
  std::vector<nav_msgs::msg::Path> candidates;
  std::vector<double> scores;

  // Compute mid point between start and goal
  geometry_msgs::msg::PoseStamped mid_point;
  mid_point.header = start.header;
  mid_point.pose.position.x = (start.pose.position.x + goal.pose.position.x) / 2.0;
  mid_point.pose.position.y = (start.pose.position.y + goal.pose.position.y) / 2.0;
  mid_point.pose.position.z = (start.pose.position.z + goal.pose.position.z) / 2.0;
  mid_point.pose.orientation = goal.pose.orientation;  // Use goal orientation for simplicity

  for (int i = 0; i < num_paths_; i++) {
    geometry_msgs::msg::PoseStamped perturbed_mid = mid_point;

    // --- perturb mid point orientation ---
    double yaw = tf2::getYaw(perturbed_mid.pose.orientation);
    yaw += (i - num_paths_ / 2) * mid_yaw_offset_;
    perturbed_mid.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(yaw / 2), cos(yaw / 2)));

    // --- perturb mid point position (lateral spread) ---
    perturbed_mid.pose.position.y += (i - num_paths_ / 2) * mid_xy_offset_;

    try {
      auto path1 = navfn_.createPlan(start, perturbed_mid);
      auto path2 = navfn_.createPlan(perturbed_mid, goal);

      if (!path1.poses.empty() && !path2.poses.empty()) {
        auto combined_path = combinePaths(path1, path2);
        candidates.push_back(combined_path);
        scores.push_back(scorePath(combined_path));

        debug_pub_->publish(combined_path);
      }
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "Navfn failed for candidate %d", i);
    }
  }

  // --- select best path ---
  int best_idx = 0;
  double best_score = std::numeric_limits<double>::max();

  for (size_t i = 0; i < scores.size(); i++) {
    if (scores[i] < best_score) {
      best_score = scores[i];
      best_idx = i;
    }
  }

  return candidates.empty() ? nav_msgs::msg::Path() : candidates[best_idx];
}

double SmacMultiPlanner::scorePath(const nav_msgs::msg::Path& path) {
  // cost-based scoring using costmap cell costs
  if (!costmap_ros_) {
    // fallback to length when costmap unavailable
    double fallback_cost = 0.0;
    for (size_t i = 1; i < path.poses.size(); i++) {
      auto& p1 = path.poses[i - 1].pose.position;
      auto& p2 = path.poses[i].pose.position;
      fallback_cost += hypot(p2.x - p1.x, p2.y - p1.y);
    }
    return fallback_cost;
  }

  auto costmap = costmap_ros_->getCostmap();
  double cost = 0.0;

  for (auto& pose_stamped : path.poses) {
    unsigned int mx, my;
    if (!costmap->worldToMap(pose_stamped.pose.position.x, pose_stamped.pose.position.y, mx, my)) {
      continue;
    }
    cost += static_cast<double>(costmap->getCost(mx, my));
  }

  return cost;
}

nav_msgs::msg::Path SmacMultiPlanner::combinePaths(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2) {
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

}  // namespace smac_multi_planner

PLUGINLIB_EXPORT_CLASS(smac_multi_planner::SmacMultiPlanner, nav2_core::GlobalPlanner)