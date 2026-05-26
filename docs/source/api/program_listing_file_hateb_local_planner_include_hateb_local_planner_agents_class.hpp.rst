
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_agents_class.hpp:

Program Listing for File agents_class.hpp
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_agents_class.hpp>` (``hateb_local_planner/include/hateb_local_planner/agents_class.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*******************************************************************************
    * Software License Agreement (MIT License)
    *
    * Copyright (c) 2020-2025 LAAS-CNRS
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
   
   #ifndef AGENTS_HH_
   #define AGENTS_HH_
   
   // ROS2 core
   // #include <hateb_local_planner/agents_config.hpp>
   #include <hateb_local_planner/hateb_config.hpp>
   #include <rclcpp/rclcpp.hpp>
   #include <ros2_helpers/parameters.hpp>
   
   // TF2
   #include <tf2/convert.h>
   #include <tf2/utils.h>
   #include <tf2_ros/buffer.h>
   #include <tf2_ros/transform_listener.h>
   
   #include <cstdint>
   
   // COSTMAP
   #include <nav2_costmap_2d/costmap_2d_ros.hpp>
   
   // MSGS
   #include <agent_path_prediction/msg/agents_info.hpp>
   #include <cohan_msgs/msg/agent_path_array.hpp>
   #include <cohan_msgs/msg/agent_trajectory.hpp>
   #include <cohan_msgs/msg/agent_trajectory_array.hpp>
   #include <cohan_msgs/msg/state_array.hpp>
   #include <cohan_msgs/msg/tracked_agents.hpp>
   #include <cohan_msgs/msg/tracked_segment_type.hpp>
   #include <geometry_msgs/msg/pose.hpp>
   #include <geometry_msgs/msg/pose2_d.hpp>
   
   // OTHERS
   #include <Eigen/Core>
   #include <map>
   #include <memory>
   #include <string>
   #include <thread>
   #include <vector>
   
   // Constants
   #define CALC_EPS 0.0001
   #define COST_MIN 250
   #define COST_OBS 255
   #define MAX_PTS 1000
   #define MIN_PTS 100
   #define AGENT_NUM_TH 5
   #define DEFAULT_AGENT_SEGMENT cohan_msgs::msg::TrackedSegmentType::TORSO
   
   namespace hateb_local_planner {
   enum AgentState { NO_STATE, STATIC, MOVING, STOPPED, BLOCKED };
   
   // Pattern 2 of ROS2 Nodes --> Use a private node instance that can be obtained from another node
   class Agents {
    public:
     Agents(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, std::shared_ptr<HATebConfig> cfg);
   
     ~Agents() {
       // rclcpp::shutdown();  // Stop spinning
       // if (spin_thread_.joinable()) {
       //   spin_thread_.join();
       // }
     }
   
     void setState(AgentState state, int id) {
       agents_states_[id] = state;
       if (state == AgentState::BLOCKED) {
         stuck_ = true;
         stuck_agent_id_ = id;
       }
     }
   
     void resetAgents();
   
     bool isAgentStuck() const { return stuck_; }
   
     void resetStuckAgent() { stuck_agent_id_ = -1; }
   
     std::map<int, geometry_msgs::msg::Pose> getAgents() { return agents_; }
   
     AgentState agentState(int id) { return agents_states_[id]; }
   
     std::map<int, double> getNominalVels() { return agent_nominal_vels_; }
   
    private:
     // Callbacks
     void trackedAgentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr tracked_agents);
   
     // Methods
     std::vector<int> filterVisibleAgents(std::map<int, geometry_msgs::msg::Pose> tr_agents, std::vector<int> sorted_ids, std::map<int, double> agents_radii, geometry_msgs::msg::Pose2D robot_pose);
   
     // Agent State Variables
     cohan_msgs::msg::TrackedAgents::SharedPtr tracked_agents_;      
     std::map<int, geometry_msgs::msg::Pose> agents_, prev_agents_;  
     std::map<int, bool> agent_still_;                               
     std::map<int, std::vector<double>> agent_vels_;                 
     std::map<int, double> agent_nominal_vels_;                      
     std::map<int, AgentState> agents_states_, prev_agents_states_;  
     std::vector<int> visible_agent_ids_;                            
   
     // Configuration and Status
     std::string tracked_agents_sub_topic_;  
     bool initialized_;                      
     bool stuck_;                            
     int stuck_agent_id_;                    
   
     // ROS2
     rclcpp_lifecycle::LifecycleNode::SharedPtr node_;                                       
     parameters::ParameterHelper param_helper_;                                              
     rclcpp::Publisher<agent_path_prediction::msg::AgentsInfo>::SharedPtr agents_info_pub_;  
     rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr tracked_agents_sub_;    
     std::shared_ptr<tf2_ros::Buffer> tf_;                                                   
     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;                            
     nav2_costmap_2d::Costmap2D* costmap_;                                                   
     double inflation_radius_;                                                               
     std::string ns_;                                                                        
     std::string map_frame_;                                                                 
     int planning_mode_;                                                                     
     std::shared_ptr<HATebConfig> cfg_;                                                      
   };
   }  // namespace hateb_local_planner
   #endif
