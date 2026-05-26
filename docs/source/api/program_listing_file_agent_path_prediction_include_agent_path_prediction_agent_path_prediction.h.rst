
.. _program_listing_file_agent_path_prediction_include_agent_path_prediction_agent_path_prediction.h:

Program Listing for File agent_path_prediction.h
================================================

|exhale_lsh| :ref:`Return to documentation for file <file_agent_path_prediction_include_agent_path_prediction_agent_path_prediction.h>` (``agent_path_prediction/include/agent_path_prediction/agent_path_prediction.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    * Copyright (c) 2020 LAAS/CNRS
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
    *  Authors: Phani Teja Singamaneni
    *********************************************************************/
   
   #ifndef AGENT_PATH_PREDICTION_H_
   #define AGENT_PATH_PREDICTION_H_
   
   #include <agent_path_prediction/AgentGoal.h>
   #include <agent_path_prediction/AgentPathPredictionConfig.h>
   #include <agent_path_prediction/AgentPose.h>
   #include <agent_path_prediction/AgentPosePredict.h>
   #include <agent_path_prediction/PredictedGoal.h>
   #include <agent_path_prediction/PredictedGoals.h>
   #include <agent_path_prediction/agents_class.h>
   #include <agent_path_prediction/predict_goal.h>
   #include <dynamic_reconfigure/Config.h>
   #include <dynamic_reconfigure/DoubleParameter.h>
   #include <dynamic_reconfigure/IntParameter.h>
   #include <dynamic_reconfigure/Reconfigure.h>
   #include <dynamic_reconfigure/server.h>
   #include <nav_msgs/GetPlan.h>
   #include <ros/ros.h>
   #include <std_srvs/Empty.h>
   #include <std_srvs/SetBool.h>
   #include <std_srvs/Trigger.h>
   #include <tf/transform_listener.h>
   #include <visualization_msgs/MarkerArray.h>
   
   #include <string>
   
   // Some fixed parameters
   #define ANG_VEL_EPS 0.001
   #define MAX_AGENT_MARKERS 1000
   #define MIN_MARKER_LIFETIME 1.0
   #define MINIMUM_COVARIANCE_MARKERS 0.1
   #define RECALC_DIST 0.5
   #define DEFAULT_AGENT_PART cohan_msgs::TrackedSegmentType::TORSO
   #define NODE_NAME "agent_path_prediction"
   
   namespace agents {
   class AgentPathPrediction {
    public:
     AgentPathPrediction() = default;
   
     ~AgentPathPrediction() = default;
   
     void initialize();
   
     void setParams(double velobs_mul, double velobs_min_rad, double velobs_max_rad, double velobs_max_rad_time, bool velobs_use_ang);
   
    private:
     // Structs
     struct AgentPathVel {
       uint64_t id;                                   // Agent ID
       nav_msgs::Path path;                           // Predicted path for the agent
       geometry_msgs::TwistWithCovariance start_vel;  // Initial velocity of the agent
     };
   
     struct AgentStartPoseVel {
       uint64_t id;                             // Agent ID
       geometry_msgs::PoseStamped pose;         // Initial pose of the agent
       geometry_msgs::TwistWithCovariance vel;  // Initial velocity of the agent
     };
   
     // ROS Publishers and Subscribers
     ros::Publisher predicted_agents_pub_;  // Publisher for predicted agent paths
     ros::Publisher front_pose_pub_;        // Publisher for front pose information
     ros::Subscriber tracked_agents_sub_;   // Subscriber for tracked agents information
     ros::Subscriber external_paths_sub_;   // Subscriber for external path information
     ros::Subscriber predicted_goal_sub_;   // Subscriber for predicted goals
   
     // ROS Services
     ros::ServiceServer predict_agents_server_;             // Server for agent prediction service
     ros::ServiceServer set_goal_srv_;                      // Server for setting agent goals
     ros::ServiceServer reset_prediction_services_server_;  // Server for resetting predictions
     ros::ServiceClient get_plan_client_;                   // Client for getting navigation plans
   
     // Transform listener
     tf::TransformListener tf_;  // Transform listener for coordinate transformations
   
     // subscriber callbacks
     void trackedAgentsCB(const cohan_msgs::TrackedAgents &tracked_agents);
   
     void externalPathsCB(const cohan_msgs::AgentPathArray::ConstPtr &external_paths);
   
     void predictedGoalCB(const agent_path_prediction::PredictedGoals::ConstPtr &predicted_goal);
   
     // Service callbacks
     bool predictAgents(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res);
   
     bool predictAgentsVelObs(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res) const;
   
     bool predictAgentsExternal(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res);
   
     bool predictAgentsBehind(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res);
   
     bool predictAgentsGoal(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res);
   
     bool predictAgentsFromPaths(agent_path_prediction::AgentPosePredict::Request &req, agent_path_prediction::AgentPosePredict::Response &res, const std::vector<AgentPathVel> &path_vels);
   
     bool setGoal(agent_path_prediction::AgentGoal::Request &req, agent_path_prediction::AgentGoal::Response &res);
   
     bool resetPredictionSrvs(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
   
     // dynamic reconfigure variables
     dynamic_reconfigure::Server<agent_path_prediction::AgentPathPredictionConfig> *dsrv_;
     void reconfigureCB(agent_path_prediction::AgentPathPredictionConfig &config, uint32_t level);
   
     // Internal Methods
     void loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh);
   
     static nav_msgs::Path setFixedPath(const geometry_msgs::PoseStamped &start_pose);
   
     static size_t prunePath(size_t begin_index, const geometry_msgs::Pose &pose, const std::vector<geometry_msgs::PoseWithCovarianceStamped> &path);
   
     bool transformPoseTwist(const cohan_msgs::TrackedAgents &tracked_agents, const uint64_t &agent_id, const std::string &to_frame, geometry_msgs::PoseStamped &pose,
                             geometry_msgs::TwistStamped &twist) const;
   
     static double checkdist(geometry_msgs::Pose agent, geometry_msgs::Pose robot) { return std::hypot(agent.position.x - robot.position.x, agent.position.y - robot.position.y); }
   
     // Properties
     cohan_msgs::TrackedAgents tracked_agents_;                                 
     cohan_msgs::AgentPathArray::ConstPtr external_paths_;                      
     agent_path_prediction::PredictedGoals predicted_goals_;                    
     std::vector<agent_path_prediction::AgentPose> external_goals_;             
     std::vector<AgentPathVel> path_vels_;                                      
     std::vector<int> path_vels_pos_;                                           
     std::vector<agent_path_prediction::PredictedPoses> last_predicted_poses_;  
     std::map<uint64_t, size_t> last_prune_indices_;                            
     std::map<uint64_t, int> last_markers_size_map_;                            
     visualization_msgs::MarkerArray predicted_agents_markers_;                 
     std::string tracked_agents_sub_topic_, external_paths_sub_topic_, predict_service_name_, predicted_agents_markers_pub_topic_, get_plan_srv_name_,
         predicted_goal_topic_;                                                   
     std::string robot_frame_id_, map_frame_id_;                                  
     double velobs_mul_, velobs_min_rad_, velobs_max_rad_, velobs_max_rad_time_;  
     double agent_dist_behind_robot_, agent_angle_behind_robot_;                  
     bool velobs_use_ang_, check_path_;                                           
     bool publish_markers_, showing_markers_,                                     
         got_new_agent_paths_, got_external_goal_;                                
     int default_agent_part_;                                                     
     geometry_msgs::Transform behind_pose_;                                       
     std::string ns_;                                                             
   };
   }  // namespace agents
   
   #endif  // AGENT_PATH_PREDICTION_H_
