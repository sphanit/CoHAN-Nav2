#ifndef HOMOTOPY_PLANNER_HPP_
#define HOMOTOPY_PLANNER_HPP_

#include <cohan_msgs/msg/agent_path.hpp>
#include <cohan_msgs/msg/agent_path_array.hpp>
#include <hateb_local_planner/msg/planning_mode.hpp>
#include <homotopy_planner/bernstein.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>

namespace homotopy_planner {

class HomotopyPlanner : public nav2_core::GlobalPlanner {
 public:
  HomotopyPlanner() = default;
  ~HomotopyPlanner() = default;

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
  hateb_local_planner::msg::PlanningMode current_planning_mode_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_plan_pub_;
  rclcpp::Publisher<cohan_msgs::msg::AgentPathArray>::SharedPtr all_homotopy_paths_pub_;
  rclcpp::Time last_update_time_;
  nav_msgs::msg::Path last_path_;
  std::vector<nav_msgs::msg::Path> last_all_paths_;
  Point last_goal_;
  bool control_points_;
  uint64_t path_id_counter_;

  void planningModeCallback(const hateb_local_planner::msg::PlanningMode::SharedPtr msg) { current_planning_mode_ = *msg; }

  bool checkCollision(const std::vector<Point>& points);
  double calculateLength(const std::vector<Point>& points);
  std::vector<Point> generateHomotopyPaths(const Point& start, const Point& goal);
  void publishAllHomotopyPaths();
  nav_msgs::msg::Path convertPointsToPath(const std::vector<Point>& points, const std::string& frame_id);
};

}  // namespace homotopy_planner

#endif