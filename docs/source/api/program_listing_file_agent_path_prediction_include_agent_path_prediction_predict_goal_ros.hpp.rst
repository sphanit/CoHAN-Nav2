
.. _program_listing_file_agent_path_prediction_include_agent_path_prediction_predict_goal_ros.hpp:

Program Listing for File predict_goal_ros.hpp
=============================================

|exhale_lsh| :ref:`Return to documentation for file <file_agent_path_prediction_include_agent_path_prediction_predict_goal_ros.hpp>` (``agent_path_prediction/include/agent_path_prediction/predict_goal_ros.hpp``)

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
   
   #include <yaml-cpp/yaml.h>
   
   #include <agent_path_prediction/agent_path_prediction_config.hpp>
   #include <agent_path_prediction/msg/predicted_goal.hpp>
   #include <agent_path_prediction/msg/predicted_goals.hpp>
   #include <agent_path_prediction/predict_goal.hpp>
   #include <cohan_msgs/msg/tracked_agent.hpp>
   #include <cohan_msgs/msg/tracked_agents.hpp>
   #include <cohan_msgs/msg/tracked_segment_type.hpp>
   #include <geometry_msgs/msg/pose_array.hpp>
   #include <rclcpp/rclcpp.hpp>
   #include <ros2_helpers/parameters.hpp>
   
   #define AGENTS_SUB_TOPIC "/tracked_agents"
   
   namespace agents {
   // Pattern 2 of ROS2 Nodes --> Use a private node instance that can be obtained from another node
   
   class PredictGoalROS {
    public:
     explicit PredictGoalROS(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<AgentPathPredictConfig> cfg) {
       node_ = node;
       cfg_ = cfg;
     }
   
     void initialize();
   
     ~PredictGoalROS() = default;
   
    private:
     void trackedAgentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr msg);
   
     bool loadGoals(const std::string& file);
   
     // ROS
     rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr agents_sub_;         
     rclcpp::Publisher<agent_path_prediction::msg::PredictedGoals>::SharedPtr goal_pub_;  
   
     // Core components
     agents::BayesianGoalPrediction predictor_;  
   
     // Data storage
     std::map<std::string, Eigen::Vector2d> goals_;    
     std::map<int, std::string> agent_goal_predicts_;  
   
     // Configuration
     int window_size_;  
   
     std::shared_ptr<rclcpp::Node> node_;           
     std::shared_ptr<AgentPathPredictConfig> cfg_;  
     std::string tracked_agents_sub_topic_;         
     std::string ns_;                               
   };
   }  // namespace agents
