
.. _program_listing_file_cohan_layers_include_cohan_layers_agent_layer.hpp:

Program Listing for File agent_layer.hpp
========================================

|exhale_lsh| :ref:`Return to documentation for file <file_cohan_layers_include_cohan_layers_agent_layer.hpp>` (``cohan_layers/include/cohan_layers/agent_layer.hpp``)

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
   
   #ifndef AGENT_LAYER_H
   #define AGENT_LAYER_H
   #include <tf2/utils.h>
   
   #include <agent_path_prediction/msg/agents_info.hpp>
   #include <cohan_layers/agent_layer_config.hpp>
   #include <cohan_msgs/msg/state_array.hpp>
   #include <cohan_msgs/msg/tracked_agents.hpp>
   #include <cohan_msgs/msg/tracked_segment_type.hpp>
   #include <map>
   #include <memory>
   #include <mutex>
   #include <nav2_costmap_2d/layer.hpp>
   #include <nav2_costmap_2d/layered_costmap.hpp>
   #include <rclcpp/rclcpp.hpp>
   #include <std_srvs/srv/set_bool.hpp>
   #include <string>
   #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
   #include <vector>
   
   namespace cohan_layers {
   class AgentLayer : public nav2_costmap_2d::Layer {
    public:
     AgentLayer() { layered_costmap_ = nullptr; }
   
     void onInitialize() override;
   
     void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
   
     void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override = 0;
   
     virtual void updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) = 0;
   
     static bool isDiscretized() { return false; }
   
    protected:
     struct AgentPoseVel {
       int track_id;  
       int type;      
       int state;     
       std_msgs::msg::Header header;
       geometry_msgs::msg::Pose pose;
       geometry_msgs::msg::Twist velocity;
     };
   
     void agentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr agents);
   
     void statesCB(const agent_path_prediction::msg::AgentsInfo::SharedPtr agents_info);
   
     void shutdownCB(const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res);
   
     static double Guassian1D(double x, double x0, double A, double varx) {
       double dx = x - x0;
       return A * exp(-pow(dx, 2.0) / (2.0 * varx));
     }
   
     static double Gaussian2D(double x, double y, double x0, double y0, double A, double varx, double vary) {
       double dx = x - x0;
       double dy = y - y0;
       double d = sqrt((dx * dx) + (dy * dy));
       double theta = atan2(dy, dx);
       double xx = d * cos(theta);
       double yy = d * sin(theta);
       return A / std::max(d, 1.0) * Guassian1D(xx, 0.0, 1.0, varx) * Guassian1D(yy, 0.0, 1.0, vary);
     }
   
     static double Gaussian2D_skewed(double x, double y, double x0, double y0, double A, double varx, double vary, double skew_ang) {
       double dx = x - x0;
       double dy = y - y0;
       double d = sqrt((dx * dx) + (dy * dy));
       double theta = atan2(dy, dx);
       double xx = d * cos(theta - skew_ang);
       double yy = d * sin(theta - skew_ang);
       return A / std::max(d, 1.0) * Guassian1D(xx, 0.0, 1.0, varx) * Guassian1D(yy, 0.0, 1.0, vary);
     }
   
     rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr agents_sub_;  
     rclcpp::Subscription<agent_path_prediction::msg::AgentsInfo>::SharedPtr agents_states_sub_;
     rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stopmap_srv_;  
     cohan_msgs::msg::TrackedAgents agents_;                           
     std::map<int, int> states_;                                       
     std::vector<AgentPoseVel> transformed_agents_;                    
     std::recursive_mutex lock_;
     bool first_time_, reset_, shutdown_;                        
     rclcpp::Time last_time_;                                    
     double last_min_x_, last_min_y_, last_max_x_, last_max_y_;  
   
     // Check which ones can be removed later
     double radius_, amplitude_, covar_, cutoff_;                           
     double robot_radius_, agent_radius_;                                   
     std::string ns_, tracked_agents_sub_topic_, agents_states_sub_topic_;  
     std::shared_ptr<AgentLayerConfig> cfg_;                                
   };
   }  // namespace cohan_layers
   
   #endif  // AGENT_LAYERS_H
