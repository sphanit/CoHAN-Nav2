
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_hateb_local_planner_ros.h:

Program Listing for File hateb_local_planner_ros.h
==================================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_hateb_local_planner_ros.h>` (``hateb_local_planner/include/hateb_local_planner/hateb_local_planner_ros.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    * Majorly modified by Phani Teja Singamaneni from 2020-2025
    * Additional changes licensed under the MIT License. See LICENSE file.
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2016
    *  TU Dortmund - Institute of Control Theory and Systems Engineering.
    *  All rights reserved.
    *
    *  Redistribution and use in source and binary forms, with or without
    *  modification, are permitted provided that the following conditions
    *  are met:
    *
    *   * Redistributions of source code must retain the above copyright
    *     notice, this list of conditions and the following disclaimer.
    *   * Redistributions in binary form must reproduce the above
    *     copyright notice, this list of conditions and the following
    *     disclaimer in the documentation and/or other materials provided
    *     with the distribution.
    *   * Neither the name of the institute nor the names of its
    *     contributors may be used to endorse or promote products derived
    *     from this software without specific prior written permission.
    *
    *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    *  POSSIBILITY OF SUCH DAMAGE.
    *
    * Author: Christoph Rösmann
    *********************************************************************/
   
   #ifndef HATEB_LOCAL_PLANNER_ROS_H_
   #define HATEB_LOCAL_PLANNER_ROS_H_
   
   #include <rclcpp/rclcpp.hpp>
   #include <rclcpp_lifecycle/lifecycle_node.hpp>
   #include <ros2_helpers/utils.hpp>
   
   // nav2 base class and utilities
   #include <hateb_local_planner/hateb_goal_checker.hpp>
   #include <nav2_core/controller.hpp>
   #include <nav2_core/exceptions.hpp>
   #include <nav2_core/goal_checker.hpp>
   #include <nav2_costmap_2d/array_parser.hpp>
   #include <nav2_costmap_2d/costmap_2d_ros.hpp>
   #include <nav2_costmap_2d/footprint_collision_checker.hpp>
   #include <nav2_util/odometry_utils.hpp>
   #include <nav2_util/robot_utils.hpp>
   
   // timed-elastic-band related classes
   #include <hateb_local_planner/optimal_planner.h>
   #include <hateb_local_planner/recovery_behaviors.h>
   
   #include <hateb_local_planner/visualization.hpp>
   
   // message types
   #include <cohan_msgs/srv/optimize.hpp>
   #include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
   #include <geometry_msgs/msg/point.hpp>
   #include <geometry_msgs/msg/pose_stamped.hpp>
   #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
   #include <geometry_msgs/msg/twist.hpp>
   #include <geometry_msgs/msg/twist_stamped.hpp>
   #include <nav_msgs/msg/odometry.hpp>
   #include <nav_msgs/msg/path.hpp>
   #include <std_msgs/msg/string.hpp>
   #include <visualization_msgs/msg/marker.hpp>
   #include <visualization_msgs/msg/marker_array.hpp>
   
   // agent data
   #include <agent_path_prediction/agent_path_prediction.hpp>
   #include <agent_path_prediction/msg/predicted_goal.hpp>
   #include <agent_path_prediction/srv/agent_pose_predict.hpp>
   #include <cohan_msgs/msg/state_array.hpp>
   #include <hateb_local_planner/agents_class.hpp>
   #include <std_srvs/srv/empty.hpp>
   #include <std_srvs/srv/set_bool.hpp>
   #include <std_srvs/srv/trigger.hpp>
   
   // transforms
   #include <tf2/convert.h>
   #include <tf2/utils.h>
   #include <tf2_ros/buffer.h>
   #include <tf2_ros/transform_listener.h>
   
   #include <tf2_eigen/tf2_eigen.hpp>
   #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
   
   // costmap converter
   #include <costmap_converter/costmap_converter_interface.h>
   
   #include <pluginlib/class_loader.hpp>
   
   // Behavior Tree and Mode Switch
   #include <hateb_local_planner/mode_switch.h>
   
   // Standard library
   #include <memory>
   #include <mutex>
   #include <string>
   #include <vector>
   
   #define OPTIMIZE_SRV_NAME "optimize"
   #define HATEB_LOG "hateb_log"
   #define THROTTLE_RATE 5.0  // seconds
   
   namespace hateb_local_planner {
   class HATebLocalPlannerROS : public nav2_core::Controller {
    public:
     HATebLocalPlannerROS() = default;
   
     ~HATebLocalPlannerROS() override = default;
   
     void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
   
     void cleanup() override;
   
     void activate() override;
   
     void deactivate() override;
   
     void setPlan(const nav_msgs::msg::Path& path) override;
   
     geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity, nav2_core::GoalChecker* goal_checker) override;
   
     void setSpeedLimit(const double& speed_limit, const bool& percentage) override;
   
     bool cancel() { return false; };
   
     bool onGoalReached();
   
   
     FootprintModelPtr getRobotFootprintFromParamServer(const rclcpp_lifecycle::LifecycleNode::SharedPtr node);
   
     Point2dContainer makeFootprintFromParams(const rclcpp::Parameter& footprint_param, const std::string& full_param_name);
   
   
    protected:
     void updateObstacleContainerWithCostmap();
   
     void updateObstacleContainerWithCostmapConverter();
   
     void updateObstacleContainerWithCustomObstacles();
   
     void updateObstacleContainerWithInvHumans();
   
     void updateViaPointsContainer(const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, double min_separation);
   
     void updateAgentViaPointsContainers(const AgentPlanVelMap& transformed_agent_plan_vel_map, double min_separation);
   
     void customObstacleCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr obst_msg);
   
     void InvHumansCB(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr obst_msg);
   
     void customViaPointsCB(const nav_msgs::msg::Path::SharedPtr via_points_msg);
   
     bool pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose, nav_msgs::msg::Path& global_plan, double dist_behind_robot = 1);
   
     bool transformGlobalPlan(const nav_msgs::msg::Path& global_plan, const geometry_msgs::msg::PoseStamped& global_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                              double max_plan_length, PlanCombined& transformed_plan_combined, int* current_goal_idx = nullptr, geometry_msgs::msg::TransformStamped* tf_plan_to_global = nullptr) const;
   
     bool transformAgentPlan(const geometry_msgs::msg::PoseStamped& robot_pose, const nav2_costmap_2d::Costmap2D& costmap, const std::string& global_frame,
                             const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& agent_plan, AgentPlanCombined& transformed_agent_plan_combined,
                             geometry_msgs::msg::TwistStamped& transformed_agent_twist, tf2::Stamped<tf2::Transform>* tf_agent_plan_to_global = nullptr) const;
   
     double estimateLocalGoalOrientation(const std::vector<geometry_msgs::msg::PoseStamped>& global_plan, const geometry_msgs::msg::PoseStamped& local_goal, int current_goal_idx,
                                         const geometry_msgs::msg::TransformStamped& tf_plan_to_global, int moving_average_length = 3);
   
     void saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards);
   
     double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0);
   
     void configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan, int& goal_idx);
   
     void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);
   
     // Agent Prediction reset
     void resetAgentsPrediction();
   
     bool tickTreeAndUpdatePlans(const geometry_msgs::msg::PoseStamped& robot_pose, std::vector<AgentPlanCombined>& transformed_agent_plans, AgentPlanVelMap& transformed_agent_plan_vel_map);
   
     bool optimizeStandalone(const std::shared_ptr<cohan_msgs::srv::Optimize::Request> req, std::shared_ptr<cohan_msgs::srv::Optimize::Response> res);
   
     bool isPassingThroughObstacle(const geometry_msgs::msg::Pose start, const geometry_msgs::msg::Pose goal, const nav2_costmap_2d::Costmap2D& costmap) const;
   
    private:
     // Definition of member variables
   
     // ROS 2 node
     rclcpp_lifecycle::LifecycleNode::WeakPtr node_;  
     std::string plugin_name_;                        
   
     // external objects (store shared pointers for ROS 2)
     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  
     nav2_costmap_2d::Costmap2D* costmap_;                         
     std::shared_ptr<tf2_ros::Buffer> tf_;                         
     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;     
   
     // internal objects (memory management owned)
     PlannerInterfacePtr planner_;                                  
     ObstContainer obstacles_;                                      
     ViaPointContainer via_points_;                                 
     std::map<uint64_t, ViaPointContainer> agents_via_points_map_;  
     TebVisualizationPtr visualization_;                            
     std::shared_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>> collision_checker_;  
     std::shared_ptr<HATebConfig> cfg_;                                                                            
     FailureDetector failure_detector_;                                                                            
   
     nav_msgs::msg::Path global_plan_;  
     std::shared_ptr<nav2_util::OdomSmoother> odom_helper_;
   
     std::unique_ptr<pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons>> costmap_converter_loader_;  
     std::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;                                 
   
     rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub_;  
     rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr inv_humans_sub_;   
     std::mutex custom_obst_mutex_;                                                                    
     std::mutex inv_human_mutex_;                                                                      
     costmap_converter_msgs::msg::ObstacleArrayMsg custom_obstacle_msg_;                               
     costmap_converter_msgs::msg::ObstacleArrayMsg inv_humans_msg_;                                    
   
     rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_points_sub_;  
     bool custom_via_points_active_;                                        
     std::mutex via_point_mutex_;                                           
   
     PoseSE2 robot_pose_;                      
     PoseSE2 robot_goal_;                      
     geometry_msgs::msg::Twist robot_vel_;     
     bool goal_reached_;                       
     bool horizon_reduced_;                    
     rclcpp::Time time_last_infeasible_plan_;  
     int no_infeasible_plans_;                 
     rclcpp::Time time_last_oscillation_;      
     RotType last_preferred_rotdir_;           
     geometry_msgs::msg::Twist last_cmd_;      
   
     std::vector<geometry_msgs::msg::Point> footprint_spec_;  
     double robot_inscribed_radius_;                          
     double robot_circumscribed_radius_;                      
   
     std::string global_frame_;      
     std::string robot_base_frame_;  
   
     bool initialized_;  
   
     // Agent prediction services and related variables
     rclcpp::Client<agent_path_prediction::srv::AgentPosePredict>::SharedPtr predict_agents_client_;  
     rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_agents_prediction_client_;                 
   
     std::string predict_srv_name_;           
     std::string reset_prediction_srv_name_;  
     std::string publish_makers_srv_name_;    
   
     rclcpp::Service<cohan_msgs::srv::Optimize>::SharedPtr optimize_server_;  
     rclcpp::Time last_call_time_;                                            
     rclcpp::Time last_omega_sign_change_;                                    
     double last_omega_;                                                      
   
     // Planning control flags
     bool goal_ctrl_;     
     bool reset_states_;  
   
     int isMode_;  
   
     std::string logs_;                                                         
     rclcpp::Subscription<cohan_msgs::msg::StateArray>::SharedPtr agents_sub_;  
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;              
     std::string invisible_humans_sub_topic_;                                   
   
     // Helper class instances
     std::shared_ptr<hateb_local_planner::Agents> agents_ptr_;  
     ModeSwitch bt_mode_switch_;                                
   
     // ROS 2 logging and time
     rclcpp::Logger logger_{rclcpp::get_logger("HATebLocalPlanner")};  
     rclcpp::Clock::SharedPtr clock_;                                  
     rclcpp::Node::SharedPtr intra_node_costmap_converter_;            
     rclcpp::Node::SharedPtr intra_node_btree_;                        
   
     // For Service Clients
     rclcpp::Node::SharedPtr client_node_;                                         
     std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   };  // end namespace hateb_local_planner
   
   #endif  // HATEB_LOCAL_PLANNER_ROS_H_
