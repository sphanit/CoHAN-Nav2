
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_hateb_config.hpp:

Program Listing for File hateb_config.hpp
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_hateb_config.hpp>` (``hateb_local_planner/include/hateb_local_planner/hateb_config.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    * Majorly modified by Phani Teja Singamaneni in 2021-2025
    * Additional changes licensed under the MIT License. See LICENSE file.
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2016,
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
    * Author: Phani Teja Singamaneni
    *********************************************************************/
   
   #ifndef HATEB_CONFIG_H_
   #define HATEB_CONFIG_H_
   
   #include <hateb_local_planner/footprint_model.h>
   
   #include <Eigen/Core>
   #include <Eigen/StdVector>
   #include <mutex>
   #include <rclcpp/rclcpp.hpp>
   #include <ros2_helpers/parameters.hpp>
   
   #define M_PI 3.14
   
   // Definitions
   #define USE_ANALYTIC_JACOBI  // if available for a specific edge, use analytic jacobi
   
   namespace hateb_local_planner {
   
   class HATebConfig {
    public:
     std::string ns;               
     std::string odom_topic;       
     std::string global_frame;     
     std::string map_frame;        
     std::string base_frame;       
     std::string footprint_frame;  
   
     FootprintModelPtr robot_model;     
     CircularFootprintPtr human_model;  
   
     std::string invisible_humans_sub_topic;  
     std::string predict_srv_name;            
     std::string reset_prediction_srv_name;   
   
     int planning_mode;       
     double planning_radius;  
   
     struct Trajectory {
       bool teb_autosize;     
       double dt_ref;         
       double dt_hysteresis;  
       int min_samples;       
       int max_samples;  
       int agent_min_samples;                   
       bool global_plan_overwrite_orientation;  
       bool allow_init_with_backwards_motion;  
       double global_plan_viapoint_sep;        
       bool via_points_ordered;                
       double max_global_plan_lookahead_dist;  
       double global_plan_prune_distance;      
       bool exact_arc_length;  
       double force_reinit_new_goal_dist;     
       double force_reinit_new_goal_angular;  
       int feasibility_check_no_poses;        
       bool publish_feedback;                 
       double min_resolution_collision_check_angular;  
       int control_look_ahead_poses;                   
       double teb_init_skip_dist;                      
       double visualize_with_time_as_z_axis_scale;     
     } trajectory;
   
     struct Robot {
       double radius;               
       double max_vel_x;            
       double max_vel_x_backwards;  
       double max_vel_y;            
       double max_vel_theta;        
       double acc_lim_x;            
       double acc_lim_y;            
       double acc_lim_theta;        
       double min_turning_radius;   
       double wheelbase;  
       bool cmd_angle_instead_rotvel;  
       bool is_footprint_dynamic;      //<! If true, updated the footprint before checking trajectory feasibility
       bool use_simulated_fov;         
   
     } robot;
   
     struct RobotFootprint {
       std::string type;
       double radius;
       std::vector<double> line_start;
       std::vector<double> line_end;
       double front_offset;
       double front_radius;
       double rear_offset;
       double rear_radius;
       std::string vertices;
   
     } robot_footprint;
   
     struct Agent {
       double radius;               
       double max_vel_x;            
       double max_vel_y;            
       double max_vel_x_backwards;  
       double min_vel_x_backwards;  
       double max_vel_theta;        
       double min_vel_theta;        
       double acc_lim_x;            
       double acc_lim_y;            
       double acc_lim_theta;        
       double fov;                  
       int num_moving_avg;          
     } agent;
   
     struct GoalTolerance {
       double yaw_goal_tolerance;  
       double xy_goal_tolerance;   
       bool free_goal_vel;         
       bool complete_global_plan;  // true prevents the robot from ending the path early when it cross the end goal
     } goal_tolerance;
   
     struct Obstacles {
       double min_obstacle_dist;  
       double inflation_dist;     
       bool use_nonlinear_obstacle_penalty;
       double obstacle_cost_mult;
       double
           dynamic_obstacle_inflation_dist;  
       bool include_dynamic_obstacles;  
       bool include_costmap_obstacles;  
       double costmap_obstacles_behind_robot_dist;  
       int obstacle_poses_affected;       
       bool legacy_obstacle_association;  
       double obstacle_association_force_inclusion_factor;  
       double obstacle_association_cutoff_factor;  
       std::string costmap_converter_plugin;       
       bool costmap_converter_spin_thread;         
       int costmap_converter_rate;  
     } obstacles;                   
   
     struct Optimization {
       int no_inner_iterations;  
       int no_outer_iterations;  
   
       bool optimization_activate;  
       bool optimization_verbose;   
   
       double penalty_epsilon;                    
       double time_penalty_epsilon;               
       bool cap_optimaltime_penalty;              
       double weight_max_vel_x;                   
       double weight_max_vel_y;                   
       double weight_max_vel_theta;               
       double weight_acc_lim_x;                   
       double weight_acc_lim_y;                   
       double weight_acc_lim_theta;               
       double weight_kinematics_nh;               
       double weight_kinematics_forward_drive;    
       double weight_kinematics_turning_radius;   
       double weight_optimaltime;                 
       double weight_shortest_path;               
       double weight_obstacle;                    
       double weight_inflation;                   
       double weight_dynamic_obstacle;            
       double weight_dynamic_obstacle_inflation;  
       double weight_viapoint;                    
       double weight_prefer_rotdir;               
       double weight_adapt_factor;  
       double obstacle_cost_exponent;         
       double weight_max_agent_vel_x;         
       double weight_max_agent_vel_y;         
       double weight_nominal_agent_vel_x;     
       double weight_max_agent_vel_theta;     
       double weight_agent_acc_lim_x;         
       double weight_agent_acc_lim_y;         
       double weight_agent_acc_lim_theta;     
       double weight_agent_optimaltime;       
       double weight_agent_viapoint;          
       double weight_invisible_human;         
       double weight_agent_robot_safety;      
       double weight_agent_agent_safety;      
       double weight_agent_robot_rel_vel;     
       double weight_agent_robot_visibility;  
       bool disable_warm_start;               
       bool disable_rapid_omega_chage;        
       double omega_chage_time_seperation;    
     } optim;                                 
   
     struct Hateb {
       bool use_agent_robot_safety_c;      
       bool use_agent_agent_safety_c;      
       bool use_agent_robot_rel_vel_c;     
       bool add_invisible_humans;          
       bool use_agent_robot_visi_c;        
       bool use_agent_elastic_vel;         
       double pose_prediction_reset_time;  
       double min_agent_robot_dist;        
       double min_agent_agent_dist;        
       double rel_vel_cost_threshold;      
       double invisible_human_threshold;   
       double visibility_cost_threshold;   
       double prediction_time_horizon;     
     } hateb;
   
     struct Recovery {
       bool shrink_horizon_backup;          
       double shrink_horizon_min_duration;  
       bool oscillation_recovery;     
       double oscillation_v_eps;      
       double oscillation_omega_eps;  
       double oscillation_recovery_min_duration;  
       double oscillation_filter_duration;        
     } recovery;                                  
   
     struct Visualization {
       bool publish_robot_global_plan;           
       bool publish_robot_local_plan;            
       bool publish_robot_local_plan_poses;      
       bool publish_robot_local_plan_fp_poses;   
       bool publish_agents_global_plans;         
       bool publish_agents_local_plans;          
       bool publish_agents_local_plan_poses;     
       bool publish_agents_local_plan_fp_poses;  
       double pose_array_z_scale;                
     } visualization;
   
     struct BTModeSwitch {
       std::string bt_xml_path;        
       std::string agents_info_topic;  
       std::string plan_sub_topic;     
       std::string result_sub_topic;   
       std::string passage_sub_topic;  
     } bt_mode_switch;
   
     HATebConfig() {
       odom_topic = "odom";
       global_frame = "map";
       map_frame = "odom";
       base_frame = "base_link";
       footprint_frame = "base_footprint";
       predict_srv_name = "/agent_path_prediction/predict_agent_poses";
       reset_prediction_srv_name = "/agent_path_prediction/reset_prediction_services";
       invisible_humans_sub_topic = "/invisible_humans_detection/invisible_humans_obs";
   
       planning_mode = 1;       // Agent-Aware planning by default
       planning_radius = 10.0;  // meters
   
       // Behavior Tree Mode Switch
       bt_mode_switch.bt_xml_path = "behavior_trees/all_modes.xml";
       bt_mode_switch.agents_info_topic = "/agents_info";
       bt_mode_switch.plan_sub_topic = "/plan";
       bt_mode_switch.result_sub_topic = "/navigate_to_pose/_action/status";
       bt_mode_switch.passage_sub_topic = "/invisible_humans_detection/passage";
       // Trajectory
       trajectory.teb_autosize = true;
       trajectory.dt_ref = 0.3;
       trajectory.dt_hysteresis = 0.1;
       trajectory.min_samples = 3;
       trajectory.agent_min_samples = 3;
       trajectory.max_samples = 500;
       trajectory.global_plan_overwrite_orientation = true;
       trajectory.allow_init_with_backwards_motion = false;
       trajectory.global_plan_viapoint_sep = -1;
       trajectory.via_points_ordered = false;
       trajectory.max_global_plan_lookahead_dist = 1;
       trajectory.global_plan_prune_distance = 1;
       trajectory.exact_arc_length = false;
       trajectory.force_reinit_new_goal_dist = 1;
       trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
       trajectory.feasibility_check_no_poses = 5;
       trajectory.publish_feedback = false;
       trajectory.min_resolution_collision_check_angular = M_PI;
       trajectory.control_look_ahead_poses = 1;
       trajectory.teb_init_skip_dist = 0.4;
       trajectory.visualize_with_time_as_z_axis_scale = 0.0;
   
       // Robot
       robot.radius = 0.47;  // Default footprint radius for PR2
       robot.max_vel_x = 0.4;
       robot.max_vel_x_backwards = 0.2;
       robot.max_vel_y = 0.0;
       robot.max_vel_theta = 0.3;
       robot.acc_lim_x = 0.5;
       robot.acc_lim_y = 0.5;
       robot.acc_lim_theta = 0.5;
       robot.min_turning_radius = 0;
       robot.wheelbase = 1.0;
       robot.cmd_angle_instead_rotvel = false;
       robot.is_footprint_dynamic = false;
       robot.use_simulated_fov = false;
   
       // Robot footprint model defaults
       robot_footprint.type = "polygon";
       robot_footprint.radius = 0.34;
       robot_footprint.line_start = {-0.3, 0.0};
       robot_footprint.line_end = {0.3, 0.0};
       robot_footprint.front_offset = 0.2;
       robot_footprint.front_radius = 0.2;
       robot_footprint.rear_offset = 0.2;
       robot_footprint.rear_radius = 0.2;
       robot_footprint.vertices = "[[-0.325, -0.325], [-0.325, 0.325], [0.325, 0.325], [0.46, 0.0], [0.325, -0.325]]";
   
       // Agent
       agent.radius = 0.3;
       agent.max_vel_x = 1.3;
       agent.max_vel_y = 0.4;
       agent.max_vel_x_backwards = 0.0;
       agent.max_vel_theta = 1.1;
       agent.acc_lim_x = 0.6;
       agent.acc_lim_theta = 0.8;
       agent.num_moving_avg = 5;
       agent.fov = 3.14;  // 180 degrees
   
       // GoalTolerance
   
       goal_tolerance.xy_goal_tolerance = 0.2;
       goal_tolerance.yaw_goal_tolerance = 0.2;
       goal_tolerance.free_goal_vel = false;
       goal_tolerance.complete_global_plan = true;
   
       // Obstacles
   
       obstacles.min_obstacle_dist = 0.5;
       obstacles.inflation_dist = 0.6;
       obstacles.use_nonlinear_obstacle_penalty = true;
       obstacles.obstacle_cost_mult = 1.0;
       obstacles.dynamic_obstacle_inflation_dist = 0.6;
       obstacles.include_dynamic_obstacles = true;
       obstacles.include_costmap_obstacles = true;
       obstacles.costmap_obstacles_behind_robot_dist = 1.5;
       obstacles.obstacle_poses_affected = 25;
       obstacles.legacy_obstacle_association = false;
       obstacles.obstacle_association_force_inclusion_factor = 1.5;
       obstacles.obstacle_association_cutoff_factor = 5;
       obstacles.costmap_converter_plugin = "";
       obstacles.costmap_converter_spin_thread = true;
       obstacles.costmap_converter_rate = 5;
   
       // Optimization
       optim.no_inner_iterations = 8;
       optim.no_outer_iterations = 4;
       optim.optimization_activate = true;
       optim.optimization_verbose = false;
       optim.penalty_epsilon = 0.1;
       optim.time_penalty_epsilon = 0.1;
       optim.cap_optimaltime_penalty = true;
       optim.weight_max_vel_x = 2;  // 1
       optim.weight_max_vel_y = 2;
       optim.weight_max_vel_theta = 1;
       optim.weight_acc_lim_x = 1;
       optim.weight_acc_lim_y = 1;
       optim.weight_acc_lim_theta = 1;
       optim.weight_kinematics_nh = 1000;
       optim.weight_kinematics_forward_drive = 1;
       optim.weight_kinematics_turning_radius = 1;
       optim.weight_optimaltime = 1;
       optim.weight_shortest_path = 0;
       optim.weight_obstacle = 50;
       optim.weight_inflation = 0.1;
       optim.weight_dynamic_obstacle = 50;
       optim.weight_dynamic_obstacle_inflation = 0.1;
       optim.weight_viapoint = 1;
       optim.weight_prefer_rotdir = 50;
   
       optim.weight_adapt_factor = 2.0;
       optim.obstacle_cost_exponent = 1.0;
   
       optim.weight_max_agent_vel_x = 2.0;
       optim.weight_nominal_agent_vel_x = 2.0;
       optim.weight_max_agent_vel_theta = 2.0;
       optim.weight_agent_acc_lim_x = 1;
       optim.weight_agent_acc_lim_theta = 1;
       optim.weight_agent_optimaltime = 1;
       optim.weight_agent_viapoint = 1;
       optim.weight_invisible_human = 1;
       optim.weight_agent_robot_safety = 20;
       optim.weight_agent_agent_safety = 20;
       optim.weight_agent_robot_rel_vel = 20;
       optim.weight_agent_robot_visibility = 20;
       optim.disable_warm_start = false;
       optim.disable_rapid_omega_chage = true;
       optim.omega_chage_time_seperation = 1.0;
   
       // Hateb
       hateb.use_agent_robot_safety_c = true;
       hateb.use_agent_agent_safety_c = true;
       hateb.use_agent_robot_rel_vel_c = true;
       hateb.add_invisible_humans = true;
       hateb.use_agent_robot_visi_c = true;
       hateb.use_agent_elastic_vel = true;
       hateb.pose_prediction_reset_time = 2.0;
       hateb.min_agent_robot_dist = 0.6;
       hateb.min_agent_agent_dist = 0.2;
       hateb.rel_vel_cost_threshold = 1.0;
       hateb.invisible_human_threshold = 1.0;
       hateb.visibility_cost_threshold = 1.0;
       hateb.prediction_time_horizon = 5.0;
   
       // Recovery
       recovery.shrink_horizon_backup = true;
       recovery.shrink_horizon_min_duration = 10;
       recovery.oscillation_recovery = true;
       recovery.oscillation_v_eps = 0.1;
       recovery.oscillation_omega_eps = 0.1;
       recovery.oscillation_recovery_min_duration = 10;
       recovery.oscillation_filter_duration = 10;
   
       // Visualization
       visualization.publish_robot_global_plan = true;
       visualization.publish_robot_local_plan = true;
       visualization.publish_robot_local_plan_poses = false;
       visualization.publish_robot_local_plan_fp_poses = false;
       visualization.publish_agents_global_plans = false;
       visualization.publish_agents_local_plans = true;
       visualization.publish_agents_local_plan_poses = false;
       visualization.publish_agents_local_plan_fp_poses = false;
       visualization.pose_array_z_scale = 1.0;
     }
   
     void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& plugin_name = "") {
       node_ = node;
       plugin_name_ = plugin_name;
       param_helper_.initialize(node);
       RCLCPP_INFO(node->get_logger(), "Initializing HATebConfig parameters for plugin: %s", plugin_name_.empty() ? "default" : plugin_name_.c_str());
     }
   
     void setupParameterCallback();
   
     void checkParameters() const;
   
     std::mutex& configMutex() { return config_mutex_; }
   
    private:
     void bindParameters();
   
     std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node_;  
     std::string plugin_name_;                              
     parameters::ParameterHelper param_helper_;             
     std::mutex config_mutex_;                              
   };
   
   }  // namespace hateb_local_planner
   
   #endif  // HATEB_CONFIG_H_
