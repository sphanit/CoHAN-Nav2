
.. _program_listing_file_agent_path_prediction_include_agent_path_prediction_agents_class.h:

Program Listing for File agents_class.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_agent_path_prediction_include_agent_path_prediction_agents_class.h>` (``agent_path_prediction/include/agent_path_prediction/agents_class.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    * Copyright (c) 2024 LAAS/CNRS
    * All rights reserved.
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
    *  Author: Phani Teja Singamaneni
    *********************************************************************/
   
   #ifndef AGENTS_HH_
   #define AGENTS_HH_
   
   // ROS
   #include <ros/ros.h>
   
   // TF2
   #include <tf2/convert.h>
   #include <tf2/utils.h>
   #include <tf2_ros/buffer.h>
   #include <tf2_ros/transform_listener.h>
   
   // COSTMAP
   #include <costmap_2d/costmap_2d_ros.h>
   
   // MSGS
   #include <agent_path_prediction/AgentsInfo.h>
   #include <cohan_msgs/AgentPathArray.h>
   #include <cohan_msgs/AgentTrajectory.h>
   #include <cohan_msgs/AgentTrajectoryArray.h>
   #include <cohan_msgs/StateArray.h>
   #include <cohan_msgs/TrackedAgents.h>
   #include <cohan_msgs/TrackedSegmentType.h>
   
   // OTHERS
   #include <Eigen/Core>
   #include <string>
   
   // Constants
   #define CALC_EPS 0.0001
   #define COST_MIN 200
   #define COST_OBS 255
   #define MAX_PTS 1000
   #define MIN_PTS 100
   #define AGENT_NUM_TH 5
   #define DEFAULT_AGENT_SEGMENT cohan_msgs::TrackedSegmentType::TORSO
   
   namespace agents {
   enum AgentState { NO_STATE, STATIC, MOVING, STOPPED, BLOCKED };
   
   class Agents {
    public:
     Agents();
   
     Agents(tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
   
     ~Agents() = default;
   
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
   
     std::map<int, geometry_msgs::Pose> getAgents() { return agents_; }
   
     AgentState agentState(int id){return agents_states_[id];}
   
     std::map<int, double> getNominalVels() { return agent_nominal_vels_; }
   
    private:
     // Callbacks
     void trackedAgentsCB(const cohan_msgs::TrackedAgents &tracked_agents);
   
     // Methods
     std::vector<int> filterVisibleAgents(std::map<int, geometry_msgs::Pose> tr_agents, std::vector<int> sorted_ids, std::map<int, double> agents_radii, geometry_msgs::Pose2D robot_pose);
   
     void loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh);
   
     // Agent State Variables
     cohan_msgs::TrackedAgents tracked_agents_;                      
     std::map<int, geometry_msgs::Pose> agents_, prev_agents_;       
     std::map<int, bool> agent_still_;                               
     std::map<int, std::vector<double>> agent_vels_;                 
     std::map<int, double> agent_nominal_vels_;                      
     std::map<int, AgentState> agents_states_, prev_agents_states_;  
     std::vector<int> visible_agent_ids_;                            
   
     // Configuration and Status
     std::string ns_;                        
     std::string tracked_agents_sub_topic_;  
     bool initialized_;                      
     bool stuck_;                            
     int stuck_agent_id_;                    
   
     // Parameters
     int window_moving_avg_;        
     int planning_mode_;            
     double human_radius_;          
     double robot_radius_;          
     double planning_radius_;       
     std::string base_link_frame_;  
     std::string map_frame_;        
     std::string odom_frame_;       
     bool use_simulated_fov_;       
   
     // ROS
     ros::Publisher agents_info_pub_;         
     ros::Subscriber tracked_agents_sub_;     
     tf2_ros::Buffer *tf_;                    
     costmap_2d::Costmap2DROS *costmap_ros_;  
     costmap_2d::Costmap2D *costmap_;         
     double inflation_radius_;                
   };
   }  // namespace agents
   #endif
