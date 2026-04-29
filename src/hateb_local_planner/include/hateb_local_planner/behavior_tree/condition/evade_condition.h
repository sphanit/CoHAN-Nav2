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
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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
   * @brief Method called to evaluate the conditione on each tick of the behavior tree
   * @return Status indicating whether condition is met
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines input ports for condition evaluation
   * @return Ports list containing agents_info and dist_max as inputs
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<agent_path_prediction::msg::AgentsInfo>("agents_info"), BT::InputPort<geometry_msgs::msg::Pose>("nearest_corner"), BT::BidirectionalPort<bool>("recovery"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("look_ahead_goal")};
  }

 private:
  /**
   * @brief Publishes visualization markers for the evasion vectors
   */
  void publishVectors();

  /**
   * @brief Helper struct for geometric calculations
   */
  struct Point {
    double x, y;
    Point(double _x, double _y) : x(_x), y(_y) {}
  };

  /**
   * @brief Gets the orientation of three points
   * @param a First point
   * @param b Second point
   * @param c Third point
   * @return 0=collinear, 1=CW, 2=CCW
   */
  int getOrient(Point a, Point b, Point c) {
    double val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
    if (val > 0) {
      return 1;
    }  // Clockwise
    if (val < 0) {
      return 2;
    }  // Counterclockwise
    return 0;
  };

  /**
   * @brief Checks if a point q lies on the segment pr
   * @param p First point of the segment
   * @param q Point to check
   * @param r Second point of the segment
   * @return true if q lies on segment pr, false otherwise
   */
  bool onSegment(Point p, Point q, Point r) {
    bool val = q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);

    return val;
  };

  /**
   * @brief Checks if two line segments intersect
   * @param p1 First point of the first segment
   * @param q1 Second point of the first segment
   * @param p2 First point of the second segment
   * @param q2 Second point of the second segment
   * @return true if segments intersect, false otherwise
   */
  bool doSegmentsIntersect(Point p1, Point q1, Point p2, Point q2) {
    int o1 = getOrient(p1, q1, p2);
    int o2 = getOrient(p1, q1, q2);
    int o3 = getOrient(p2, q2, p1);
    int o4 = getOrient(p2, q2, q1);

    // 1. General case: segments straddle each other
    if (o1 != o2 && o3 != o4) {
      return true;
    }

    // 2. Special cases: handles touching endpoints or overlapping collinear segments
    if (o1 == 0 && onSegment(p1, p2, q1)) {
      return true;
    }
    if (o2 == 0 && onSegment(p1, q2, q1)) {
      return true;
    }
    if (o3 == 0 && onSegment(p2, p1, q2)) {
      return true;
    }
    if (o4 == 0 && onSegment(p2, q1, q2)) {
      return true;
    }

    return false;
  }

  // Blackboard entries
  agent_path_prediction::msg::AgentsInfo agents_info_;  //!< Information about agents in the environment
  geometry_msgs::msg::Pose nearest_corner_;             //!< Position of the nearest corner
  double mid_x_;                                        //!< Midpoint x-coordinate between human and corner
  double mid_y_;                                        //!< Midpoint y-coordinate between human and corner
  std::pair<double, double> r_dx_dy_;                   //!< Pair to store the evasion direction vector (dx, dy)
  bool evasion_triggered_;                              //!< Flag to indicate if evasion has been triggered
  geometry_msgs::msg::PoseStamped goal_;                //!< Current navigation goal

  // // ROS
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;                                //!< ROS node for publishing
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vector_pub_;  //!< Publisher for vector markers

  // name of the node
  std::string name_;  //!< Name of the condition node
};
};  // namespace hateb_local_planner
