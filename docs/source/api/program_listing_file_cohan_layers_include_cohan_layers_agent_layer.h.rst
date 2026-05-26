
.. _program_listing_file_cohan_layers_include_cohan_layers_agent_layer.h:

Program Listing for File agent_layer.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_cohan_layers_include_cohan_layers_agent_layer.h>` (``cohan_layers/include/cohan_layers/agent_layer.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /**********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    * Copyright (c) 2025 LAAS/CNRS
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
    * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
    *********************************************************************/
   
   #ifndef AGENT_LAYER_H
   #define AGENT_LAYER_H
   #include <agent_path_prediction/AgentsInfo.h>
   #include <cohan_msgs/StateArray.h>
   #include <cohan_msgs/TrackedAgents.h>
   #include <cohan_msgs/TrackedSegmentType.h>
   #include <costmap_2d/layer.h>
   #include <costmap_2d/layered_costmap.h>
   #include <dynamic_reconfigure/server.h>
   #include <ros/ros.h>
   #include <std_srvs/SetBool.h>
   #include <tf2/utils.h>
   #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
   
   #include <boost/thread.hpp>
   
   namespace cohan_layers {
   class AgentLayer : public costmap_2d::Layer {
    public:
     AgentLayer() { layered_costmap_ = nullptr; }
   
     void onInitialize() override;
   
     void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
   
     void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override = 0;
   
     virtual void updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) = 0;
   
     static bool isDiscretized() { return false; }
   
    protected:
     struct AgentPoseVel {
       int track_id;  
       int type;      
       int state;     
       std_msgs::Header header;
       geometry_msgs::Pose pose;
       geometry_msgs::Twist velocity;
     };
   
     void agentsCB(const cohan_msgs::TrackedAgents& agents);
   
     void statesCB(const agent_path_prediction::AgentsInfo& agents_info);
   
     bool shutdownCB(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);
   
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
   
     ros::Subscriber agents_sub_, agents_states_sub_;  
     ros::ServiceServer stopmap_srv_;                  
     cohan_msgs::TrackedAgents agents_;                
     std::map<int, int> states_;                       
     std::vector<AgentPoseVel> transformed_agents_;    
     boost::recursive_mutex lock_;
     bool first_time_, reset_, shutdown_;                                   
     ros::Time last_time_;                                                  
     double last_min_x_, last_min_y_, last_max_x_, last_max_y_;             
     double radius_, amplitude_, covar_, cutoff_;                           
     double robot_radius_, agent_radius_;                                   
     std::string ns_, tracked_agents_sub_topic_, agents_states_sub_topic_;  
   };
   }  // namespace cohan_layers
   
   #endif  // AGENT_LAYERS_H
