#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <homotopy_planner/homotopy_planner.hpp>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace homotopy_planner {

void HomotopyPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  costmap_ros_ = costmap_ros;
  navfn_.configure(node_, name_ + ".navfn", tf, costmap_ros);
  planning_mode_sub_ = node_->create_subscription<hateb_local_planner::msg::PlanningMode>("planning_mode", 10, std::bind(&HomotopyPlanner::planningModeCallback, this, std::placeholders::_1));
  evasion_control_point_sub_ =
      node_->create_subscription<geometry_msgs::msg::Point>("evasion_control_point", 10, std::bind(&HomotopyPlanner::evasionControlPointCallback, this, std::placeholders::_1));
  valid_plan_pub_ = node_->create_publisher<std_msgs::msg::Bool>(name_ + "/valid_plan", 10);
  all_homotopy_paths_pub_ = node_->create_publisher<cohan_msgs::msg::AgentPathArray>(name_ + "/all_homotopy_paths", 10);
  last_update_time_ = node_->get_clock()->now();
  control_points_ = true;
  path_id_counter_ = 0;
  last_goal_.x = 999.0;
  last_goal_.y = 999.0;
  fail_count_ = 0;
}

void HomotopyPlanner::cleanup() { navfn_.cleanup(); }
void HomotopyPlanner::activate() { navfn_.activate(); }
void HomotopyPlanner::deactivate() { navfn_.deactivate(); }

bool HomotopyPlanner::checkCollision(const std::vector<Point>& points) {
  auto costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return false;
  }

  double res = costmap->getResolution();
  auto origin = costmap->getOriginX();
  double oy = costmap->getOriginY();
  uint32_t w = costmap->getSizeInCellsX();
  uint32_t h = costmap->getSizeInCellsY();

  for (const auto& p : points) {
    int gx = static_cast<int>((p.x - origin) / res);
    int gy = static_cast<int>((p.y - oy) / res);

    if (gx >= 0 && gx < static_cast<int>(w) && gy >= 0 && gy < static_cast<int>(h)) {
      unsigned char cost = costmap->getCost(gx, gy);
      if (cost > 100) {  // Lethal obstacle
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

double HomotopyPlanner::calculateLength(const std::vector<Point>& points) {
  double length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    length += (points[i] - points[i - 1]).norm();
  }
  return length;
}

std::vector<HomotopyPlanner::Point> HomotopyPlanner::generateHomotopyPaths(const Point& start, const Point& goal) {
  std::vector<std::vector<Point>> left_paths;   // negative offsets
  std::vector<std::vector<Point>> right_paths;  // positive offsets

  // Clear previous paths
  last_all_paths_.clear();

  Point direction = (goal - start);
  double dist_total = direction.norm();

  if (dist_total < 0.1) {
    return std::vector<Point>();
  }

  Point unit_dir = direction * (1.0 / dist_total);
  Point normal(-unit_dir.y, unit_dir.x);

  // Make this configurable in the future if needed
  // TODO: Make these numbers configurable
  std::vector<double> offsets;
  for (int i = 0; i < 40; ++i) {
    offsets.push_back(-4.0 + i * (8.0 / 39.0));
  }

  for (double offset : offsets) {
    // Create control points for Bezier curve // TODO: Make the control point generation configurable
    // Point p1;
    // Point p2;
    // if (!control_points_) {
    //   p1 = start + direction * 0.1 + normal * offset;
    //   p2 = start + direction * 0.3 + normal * offset;
    //   control_points_ = true;
    // } else {
    //   p1 = start - direction * 0.5 + normal * offset;
    //   p2 = start + direction * 0.3 + normal * offset;
    //   control_points_ = false;
    // }
    Point p1 = start - direction * fail_count_ * 0.1 + normal * offset;
    Point p2 = evasion_control_point_ + normal * offset;  // Use evasion control point as second control point

    std::vector<Point> pts = {start, p1, p2, goal};
    std::vector<Point> path = bernstein_curve(pts, 50);

    last_all_paths_.push_back(convertPointsToPath(path, "map"));

    if (checkCollision(path)) {
      bool is_unique = true;
      Point mid_new = path[path.size() / 2];

      // Check uniqueness against all existing paths
      for (const auto& existing : left_paths) {
        Point mid_ext = existing[existing.size() / 2];
        if ((mid_new - mid_ext).norm() < 0.2) {
          is_unique = false;
          break;
        }
      }
      if (is_unique) {
        for (const auto& existing : right_paths) {
          Point mid_ext = existing[existing.size() / 2];
          if ((mid_new - mid_ext).norm() < 0.2) {
            is_unique = false;
            break;
          }
        }
      }

      if (is_unique) {
        auto length = calculateLength(path);
        RCLCPP_DEBUG(node_->get_logger(), "Valid Path Found - Offset: %5.2f | Length: %6.2f m", offset, length);

        if (offset < 0) {
          left_paths.push_back(path);
        } else {
          right_paths.push_back(path);
        }
      }
    }
  }

  int left_count = left_paths.size();
  int right_count = right_paths.size();

  RCLCPP_INFO(node_->get_logger(), "Found %d left-side paths and %d right-side paths", left_count, right_count);

  if (left_count > right_count && left_count > 0) {
    return left_paths[left_count / 4];  // Return middle path from dominant side
  } else if (right_count > left_count && right_count > 0) {
    return right_paths[right_count / 4];  // Return middle path from dominant side
  } else if (right_count > 0) {
    return right_paths[right_count / 4];
  } else if (left_count > 0) {
    return left_paths[left_count / 4];
  }

  // No valid paths found
  RCLCPP_WARN(node_->get_logger(), "No valid homotopy paths found");
  return std::vector<Point>();
}

void HomotopyPlanner::publishAllHomotopyPaths() {
  RCLCPP_INFO(node_->get_logger(), "Publishing all homotopy paths, count: %zu", last_all_paths_.size());
  if (!all_homotopy_paths_pub_) {
    RCLCPP_WARN(node_->get_logger(), "Publisher not initialized!");
    return;
  }

  if (last_all_paths_.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "No paths to publish");
    return;
  }

  cohan_msgs::msg::AgentPathArray agent_path_array;
  agent_path_array.header.stamp = node_->get_clock()->now();
  agent_path_array.header.frame_id = "map";

  for (size_t i = 0; i < last_all_paths_.size(); ++i) {
    cohan_msgs::msg::AgentPath agent_path;
    agent_path.id = i;
    agent_path.path = last_all_paths_[i];
    agent_path_array.paths.push_back(agent_path);
  }

  all_homotopy_paths_pub_->publish(agent_path_array);
  RCLCPP_INFO(node_->get_logger(), "Published %zu homotopy paths on topic", last_all_paths_.size());
}

nav_msgs::msg::Path HomotopyPlanner::convertPointsToPath(const std::vector<Point>& points, const std::string& frame_id) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = node_->get_clock()->now();

  for (const auto& p : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = node_->get_clock()->now();
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

// nav_msgs::msg::Path HomotopyPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) {
//   if ((last_goal_ - Point(goal.pose.position.x, goal.pose.position.y)).norm() > 0.1) {
//     current_planning_mode_.plan_mode = 0;  // Reset to default planning mode
//     last_path_.poses.clear();
//     last_all_paths_.clear();
//     last_goal_ = Point(goal.pose.position.x, goal.pose.position.y);
//     fail_count_ = 0;
//   }

//   if (static_cast<int>(current_planning_mode_.plan_mode) == 4) {
//     if (!last_path_.poses.empty()) {
//       RCLCPP_INFO(node_->get_logger(), "Using cached homotopy path");
//       // Publish cached paths again
//       publishAllHomotopyPaths();
//       return last_path_;
//     }
//     // If plan_mode is Evasion, generate homotopy Bezier curves
//     else {
//       Point start_pt(start.pose.position.x, start.pose.position.y);
//       Point goal_pt(goal.pose.position.x, goal.pose.position.y);
//       RCLCPP_INFO(node_->get_logger(), "Generating homotopy paths for start: (%.2f, %.2f) and goal: (%.2f, %.2f)", start_pt.x, start_pt.y, goal_pt.x, goal_pt.y);
//       auto best_path = generateHomotopyPaths(start_pt, goal_pt);

//       // Publish all generated homotopy paths
//       publishAllHomotopyPaths();

//       if (!best_path.empty()) {
//         std_msgs::msg::Bool msg;
//         msg.data = true;
//         valid_plan_pub_->publish(msg);
//         last_path_ = convertPointsToPath(best_path, start.header.frame_id);
//         last_update_time_ = node_->get_clock()->now();
//         last_goal_ = goal_pt;
//         fail_count_ = 0;  // Reset fail count on successful path generation
//         return last_path_;
//       } else {
//         std_msgs::msg::Bool msg;
//         msg.data = false;
//         valid_plan_pub_->publish(msg);
//         fail_count_++;
//         RCLCPP_WARN(node_->get_logger(), "No valid homotopy paths found, falling back to navfn");
//       }
//     }
//   }
//   // Default: use navfn planner
//   return navfn_.createPlan(start, goal);
// }

nav_msgs::msg::Path HomotopyPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) {
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
      RCLCPP_INFO(node_->get_logger(), "Using cached homotopy path");
      // Publish cached paths again
      publishAllHomotopyPaths();
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

nav_msgs::msg::Path HomotopyPlanner::combinePaths(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2) {
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

}  // namespace homotopy_planner

PLUGINLIB_EXPORT_CLASS(homotopy_planner::HomotopyPlanner, nav2_core::GlobalPlanner)