#ifndef SMAC_MULTI_PLANNER_HPP_
#define SMAC_MULTI_PLANNER_HPP_

#include <nav2_core/global_planner.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <nav_msgs/msg/path.hpp>

namespace smac_multi_planner {

class SmacMultiPlanner : public nav2_core::GlobalPlanner {
 public:
  SmacMultiPlanner() = default;
  ~SmacMultiPlanner() = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) override;

 private:
  // nav2_smac_planner::SmacPlannerHybrid smac_;
  nav2_navfn_planner::NavfnPlanner navfn_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;

  int num_paths_;
  double goal_yaw_offset_;
  double goal_xy_offset_;
  double mid_yaw_offset_;
  double mid_xy_offset_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_pub_;

  double scorePath(const nav_msgs::msg::Path& path);
  nav_msgs::msg::Path combinePaths(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2);
};

}  // namespace smac_multi_planner

#endif