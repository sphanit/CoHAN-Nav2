#ifndef EVASION_PLANNER_HPP_
#define EVASION_PLANNER_HPP_

#include <cohan_msgs/msg/agent_path.hpp>
#include <hateb_local_planner/msg/planning_mode.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

namespace evasion_planner {

class EvasionPlanner : public nav2_core::GlobalPlanner {
 public:
  EvasionPlanner() = default;
  ~EvasionPlanner() = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) override;

 private:
  struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x, double y) : x(x), y(y) {}

    Point operator+(const Point& p) const { return Point(x + p.x, y + p.y); }
    Point operator-(const Point& p) const { return Point(x - p.x, y - p.y); }
    Point operator*(double scalar) const { return Point(x * scalar, y * scalar); }
    Point operator/(double scalar) const { return Point(x / scalar, y / scalar); }
    double norm() const { return std::sqrt(x * x + y * y); }
  };

  nav2_navfn_planner::NavfnPlanner navfn_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Subscription<hateb_local_planner::msg::PlanningMode>::SharedPtr planning_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr evasion_control_point_sub_;
  hateb_local_planner::msg::PlanningMode current_planning_mode_;
  nav_msgs::msg::Path last_path_;
  Point last_goal_;
  Point evasion_control_point_;

  void planningModeCallback(const hateb_local_planner::msg::PlanningMode::SharedPtr msg) { current_planning_mode_ = *msg; }
  void evasionControlPointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    // Store the evasion control point for use in path generation
    evasion_control_point_ = Point(msg->x, msg->y);
  }
  nav_msgs::msg::Path combinePaths(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2);
};

}  // namespace evasion_planner

#endif