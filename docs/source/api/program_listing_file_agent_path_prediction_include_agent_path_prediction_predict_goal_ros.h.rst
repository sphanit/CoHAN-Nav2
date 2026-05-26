
.. _program_listing_file_agent_path_prediction_include_agent_path_prediction_predict_goal_ros.h:

Program Listing for File predict_goal_ros.h
===========================================

|exhale_lsh| :ref:`Return to documentation for file <file_agent_path_prediction_include_agent_path_prediction_predict_goal_ros.h>` (``agent_path_prediction/include/agent_path_prediction/predict_goal_ros.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
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
    *  Author: Phani Teja Singamaneni
    *********************************************************************/
   
   #include <agent_path_prediction/PredictedGoal.h>
   #include <agent_path_prediction/PredictedGoals.h>
   #include <agent_path_prediction/predict_goal.h>
   #include <cohan_msgs/TrackedAgent.h>
   #include <cohan_msgs/TrackedAgents.h>
   #include <cohan_msgs/TrackedSegmentType.h>
   #include <geometry_msgs/PoseArray.h>
   #include <ros/ros.h>
   #include <yaml-cpp/yaml.h>
   
   namespace agents {
   class PredictGoalROS {
    public:
     PredictGoalROS();
   
     ~PredictGoalROS() = default;
   
    private:
     void trackedAgentsCB(const cohan_msgs::TrackedAgents::ConstPtr& msg);
   
     bool loadGoals(const std::string& file);
   
     // Internal Methods
     void loadRosParamFromNodeHandle(const ros::NodeHandle& private_nh);
   
     // ROS
     ros::Subscriber agents_sub_;  
     ros::Publisher goal_pub_;     
   
     // Core components
     agents::BayesianGoalPrediction predictor_;  
   
     // Data storage
     std::map<std::string, Eigen::Vector2d> goals_;    
     std::map<int, std::string> agent_goal_predicts_;  
   
     // Configuration
     int window_size_;  
   
     std::string tracked_agents_sub_topic_, predicted_goal_topic_;  
     std::string ns_;                                               
     std::string goals_file_;                                       
   };
   }  // namespace agents
