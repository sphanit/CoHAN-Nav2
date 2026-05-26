
.. _program_listing_file_agent_path_prediction_include_agent_path_prediction_agent_path_prediction.hpp:

Program Listing for File agent_path_prediction.hpp
==================================================

|exhale_lsh| :ref:`Return to documentation for file <file_agent_path_prediction_include_agent_path_prediction_agent_path_prediction.hpp>` (``agent_path_prediction/include/agent_path_prediction/agent_path_prediction.hpp``)

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
   
   #ifndef AGENT_PATH_PREDICTION_H_
   #define AGENT_PATH_PREDICTION_H_
   
   // ROS2 core
   #include <rclcpp/rclcpp.hpp>
   #include <ros2_helpers/parameters.hpp>
   #include <ros2_helpers/utils.hpp>
   
   //  TF2
   #include <tf2/convert.h>
   #include <tf2/time.h>
   #include <tf2/utils.h>
   #include <tf2_ros/buffer.h>
   #include <tf2_ros/transform_listener.h>
   
   // Local and generic headers
   #include <agent_path_prediction/agent_path_prediction_config.hpp>
   #include <agent_path_prediction/predict_goal.hpp>
   #include <chrono>
   #include <functional>
   #include <memory>
   #include <string>
   
   // Messages
   #include <tf2/LinearMath/Quaternion.h>
   #include <tf2/LinearMath/Transform.h>
   #include <tf2/LinearMath/Vector3.h>
   
   #include <agent_path_prediction/msg/predicted_goals.hpp>
   #include <cohan_msgs/msg/agent_path_array.hpp>
   #include <cohan_msgs/msg/tracked_agents.hpp>
   #include <cohan_msgs/msg/tracked_segment_type.hpp>
   #include <geometry_msgs/msg/pose.hpp>
   #include <geometry_msgs/msg/pose_stamped.hpp>
   #include <geometry_msgs/msg/twist_with_covariance.hpp>
   #include <nav_msgs/msg/path.hpp>
   #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
   #include <visualization_msgs/msg/marker_array.hpp>
   
   // Services
   #include <agent_path_prediction/srv/agent_goal.hpp>
   #include <agent_path_prediction/srv/agent_pose_predict.hpp>
   #include <std_srvs/srv/empty.hpp>
   #include <std_srvs/srv/set_bool.hpp>
   #include <std_srvs/srv/trigger.hpp>
   
   // Actions
   #include <nav2_msgs/action/compute_path_to_pose.hpp>
   #include <rclcpp_action/rclcpp_action.hpp>
   
   // Internal Parameters
   #define ANG_VEL_EPS 0.001
   #define MAX_AGENT_MARKERS 1000
   #define MIN_MARKER_LIFETIME 1.0
   #define MINIMUM_COVARIANCE_MARKERS 0.1
   #define RECALC_DIST 0.5
   #define NODE_NAME "agent_path_prediction"
   
   namespace agents {
   // Pattern 1 of ROS2 Nodes --> Inherit from rclcpp::Node (Good for Standalone nodes)
   class AgentPathPrediction : public rclcpp::Node {
    public:
     AgentPathPrediction() : Node(NODE_NAME) {}
   
     ~AgentPathPrediction() = default;
   
     void initialize();
   
     // ROS2 node and helpers
     std::shared_ptr<AgentPathPredictConfig> cfg_;  
   
    private:
     // Structs
     struct AgentPathVel {
       uint64_t id;                                        // Agent ID
       nav_msgs::msg::Path path;                           // Predicted path for the agent
       geometry_msgs::msg::TwistWithCovariance start_vel;  // Initial velocity of the agent
     };
   
     struct AgentStartPoseVel {
       uint64_t id;                                  // Agent ID
       geometry_msgs::msg::PoseStamped pose;         // Initial pose of the agent
       geometry_msgs::msg::TwistWithCovariance vel;  // Initial velocity of the agent
     };
   
     // ROS2 Action Clients
     using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
     rclcpp_action::Client<ComputePathToPose>::SharedPtr get_plan_client_;                    
     using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;  
   
     // Action Goal Handlers
     void sendActionGoal(ComputePathToPose::Goal goal_msg);
   
     void goalResponseCB(GoalHandleComputePathToPose::SharedPtr goal_handle);
   
     void resultCB(const GoalHandleComputePathToPose::WrappedResult& result);
   
     bool isPlanningDone() const { return planning_done_; }
   
     // subscriber callbacks
     void trackedAgentsCB(const cohan_msgs::msg::TrackedAgents::SharedPtr tracked_agents);
   
     void externalPathsCB(const cohan_msgs::msg::AgentPathArray::SharedPtr external_paths);
   
     void predictedGoalCB(const agent_path_prediction::msg::PredictedGoals::SharedPtr predicted_goal);
   
     // Service callbacks
     void predictAgents(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
   
     void predictAgentsVelObs(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res) const;
   
     void predictAgentsExternal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
   
     void predictAgentsBehind(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
   
     void predictAgentsGoal(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
   
     void predictAgentsFromPaths(const std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentPosePredict::Response> res);
   
     void resetPredictionSrvs(const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res);
   
     void setGoal(const std::shared_ptr<agent_path_prediction::srv::AgentGoal::Request> req, std::shared_ptr<agent_path_prediction::srv::AgentGoal::Response> res);
   
     static nav_msgs::msg::Path setFixedPath(const geometry_msgs::msg::PoseStamped& start_pose);
   
     static size_t prunePath(size_t begin_index, const geometry_msgs::msg::Pose& pose, const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& path);
   
     bool transformPoseTwist(const cohan_msgs::msg::TrackedAgents& tracked_agents, const uint64_t& agent_id, const std::string& to_frame, geometry_msgs::msg::PoseStamped& pose,
                             geometry_msgs::msg::TwistStamped& twist) const;
   
     static double checkdist(geometry_msgs::msg::Pose agent, geometry_msgs::msg::Pose robot) { return std::hypot(agent.position.x - robot.position.x, agent.position.y - robot.position.y); }
   
     // ROS2 message handlers
     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr predicted_agents_pub_;         
     rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr front_pose_pub_;                    
     rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr tracked_agents_sub_;              
     rclcpp::Subscription<cohan_msgs::msg::AgentPathArray>::SharedPtr external_paths_sub_;             
     rclcpp::Subscription<agent_path_prediction::msg::PredictedGoals>::SharedPtr predicted_goal_sub_;  
   
     // ROS2 Services
     rclcpp::Service<agent_path_prediction::srv::AgentPosePredict>::SharedPtr predict_agents_server_;  
     rclcpp::Service<agent_path_prediction::srv::AgentGoal>::SharedPtr set_goal_srv_;                  
     rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_prediction_services_server_;               
   
     // Transform listener
     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;      
     std::shared_ptr<tf2_ros::TransformListener> tf_;  
   
     // Internal variables
     cohan_msgs::msg::TrackedAgents tracked_agents_;                                 
     cohan_msgs::msg::AgentPathArray external_paths_;                                
     agent_path_prediction::msg::PredictedGoals predicted_goals_;                    
     std::vector<agent_path_prediction::msg::AgentPose> external_goals_;             
     std::vector<AgentPathVel> path_vels_;                                           
     std::vector<int> path_vels_pos_;                                                
     std::vector<agent_path_prediction::msg::PredictedPoses> last_predicted_poses_;  
     std::map<uint64_t, size_t> last_prune_indices_;                                 
     std::map<uint64_t, int> last_markers_size_map_;                                 
     visualization_msgs::msg::MarkerArray predicted_agents_markers_;                 
     bool check_path_;                                                               
     bool showing_markers_, got_new_agent_paths_, got_external_goal_;                
     geometry_msgs::msg::Transform behind_pose_;                                     
     std::string tracked_agents_sub_topic_;                                          
     std::string get_plan_srv_name_;                                                 
     std::string ns_;                                                                
     bool planning_done_;                                                            
     nav_msgs::msg::Path planned_path_;                                              
     rclcpp::Node::SharedPtr client_node_;                                           
     std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;    
   };
   }  // namespace agents
   
   #endif  // AGENT_PATH_PREDICTION_H_
