
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_visualization.h:

Program Listing for File visualization.h
========================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_visualization.h>` (``hateb_local_planner/include/hateb_local_planner/visualization.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2016,
    *  TU Dortmund - Institute of Control Theory and Systems Engineering.
    *  All rights reserved.
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
    * Author: Christoph Rösmann
    * Modified by: Phani Teja Singamaneni
    *********************************************************************/
   
   #ifndef VISUALIZATION_H_
   #define VISUALIZATION_H_
   
   // teb stuff
   #include <hateb_local_planner/footprint_model.h>
   #include <hateb_local_planner/hateb_config.h>
   #include <hateb_local_planner/timed_elastic_band.h>
   
   // ros stuff
   #include <base_local_planner/goal_functions.h>
   #include <ros/publisher.h>
   #include <tf/transform_listener.h>
   
   // boost
   #include <boost/graph/adjacency_list.hpp>
   #include <boost/graph/graph_traits.hpp>
   
   // messages
   #include <cohan_msgs/AgentPathArray.h>
   #include <cohan_msgs/AgentTimeToGoal.h>
   #include <cohan_msgs/AgentTimeToGoalArray.h>
   #include <cohan_msgs/AgentTrajectoryArray.h>
   #include <cohan_msgs/TrackedAgents.h>
   #include <cohan_msgs/TrackedSegmentType.h>
   #include <cohan_msgs/TrajectoryPoint.h>
   #include <cohan_msgs/TrajectoryStamped.h>
   #include <geometry_msgs/PoseArray.h>
   #include <geometry_msgs/PoseStamped.h>
   #include <nav_msgs/Odometry.h>
   #include <nav_msgs/Path.h>
   #include <std_msgs/ColorRGBA.h>
   #include <std_msgs/Float32.h>
   #include <tf/transform_datatypes.h>
   #include <visualization_msgs/Marker.h>
   #include <visualization_msgs/MarkerArray.h>
   
   namespace hateb_local_planner {
   
   typedef struct {
     std::vector<geometry_msgs::PoseStamped> plan_before;
     std::vector<cohan_msgs::TrajectoryPoint> optimized_trajectory;
     std::vector<geometry_msgs::PoseStamped> plan_after;
   } PlanTrajCombined;
   
   typedef struct {
     std::vector<geometry_msgs::PoseStamped> plan_before;
     std::vector<geometry_msgs::PoseStamped> plan_to_optimize;
     std::vector<geometry_msgs::PoseStamped> plan_after;
   } PlanCombined;
   
   typedef struct {
     uint64_t id;
     std::vector<geometry_msgs::PoseStamped> plan_before;
     std::vector<cohan_msgs::TrajectoryPoint> optimized_trajectory;
     std::vector<geometry_msgs::PoseStamped> plan_after;
   } AgentPlanTrajCombined;
   
   typedef struct {
     uint64_t id;
     std::vector<geometry_msgs::PoseStamped> plan_before;
     std::vector<geometry_msgs::PoseStamped> plan_to_optimize;
     std::vector<geometry_msgs::PoseStamped> plan_after;
   } AgentPlanCombined;
   
   class TebOptimalPlanner;  
   
   class TebVisualization {
    public:
     TebVisualization();
   
     TebVisualization(ros::NodeHandle& nh, const HATebConfig& cfg);
   
     void initialize(ros::NodeHandle& nh, const HATebConfig& cfg);
   
   
     void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) const;
   
     void publishAgentGlobalPlans(const std::vector<AgentPlanCombined>& agents_plans) const;
   
     void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& local_plan) const;
   
     void publishTrackedAgents(const cohan_msgs::TrackedAgentsConstPtr& agents);
   
     void publishLocalPlanAndPoses(const TimedElasticBand& teb, const BaseFootprintModel& robot_model, double fp_size, const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));
   
     void publishAgentLocalPlansAndPoses(const std::map<uint64_t, TimedElasticBand>& agents_tebs_map, const BaseFootprintModel& agent_model, double fp_size,
                                         const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));
   
     void publishTrajectory(const PlanTrajCombined& plan_traj_combined);
   
     void publishAgentTrajectories(const std::vector<AgentPlanTrajCombined>& agents_plans_combined);
   
     void publishRobotFootprintModel(const PoseSE2& current_pose, const BaseFootprintModel& robot_model, const std::string& ns = "RobotFootprintModel",
                                     const std_msgs::ColorRGBA& color = toColorMsg(0.5, 0.0, 0.8, 0.0));
   
     void publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseFootprintModel& robot_model);
   
     void publishObstacles(const ObstContainer& obstacles) const;
   
     void publishViaPoints(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns = "ViaPoints") const;
   
     template <typename GraphType>
     void publishGraph(const GraphType& graph, const std::string& ns_prefix = "Graph");
   
     template <typename BidirIter>
     void publishPathContainer(BidirIter first, BidirIter last, const std::string& ns = "PathContainer");
   
     void publishTebContainer(const std::vector<boost::shared_ptr<TebOptimalPlanner> >& teb_planner, const std::string& ns = "TebContainer");
   
     void publishFeedbackMessage(const std::vector<boost::shared_ptr<TebOptimalPlanner> >& teb_planners, unsigned int selected_trajectory_idx, const ObstContainer& obstacles);
   
     void publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles);
   
   
     static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);
   
     static void setMarkerColour(visualization_msgs::Marker& marker, double itr, double n);
     void publishMode(int Mode);
   
    protected:
     bool printErrorWhenNotInitialized() const;
   
     void clearingTimerCB(const ros::TimerEvent& event);
   
     ros::Publisher global_plan_pub_;                                   
     ros::Publisher local_plan_pub_;                                    
     ros::Publisher local_traj_pub_;                                    
     ros::Publisher agents_global_plans_pub_;                           
     ros::Publisher agents_local_plans_pub_;                            
     ros::Publisher agents_local_trajs_pub_;                            
     ros::Publisher teb_poses_pub_, teb_fp_poses_pub_;                  
     ros::Publisher agents_tebs_poses_pub_, agents_tebs_fp_poses_pub_;  
     ros::Publisher teb_marker_pub_;                                    
     ros::Publisher feedback_pub_;                                      
     ros::Publisher mode_text_pub_;                                     
     ros::Publisher robot_traj_time_pub_;                               
     ros::Publisher robot_path_time_pub_;                               
     ros::Publisher robot_next_pose_pub_;                               
     ros::Publisher agent_next_pose_pub_;                               
     ros::Publisher agent_trajs_time_pub_;                              
     ros::Publisher agent_paths_time_pub_;                              
     ros::Publisher agent_marker_pub_;                                  
     ros::Publisher agent_arrow_pub_;                                   
     ros::Subscriber tracked_agents_sub_;                               
     std::vector<double> vel_robot_;                                    
     std::vector<double> vel_agent_;                                    
     tf::TransformListener tf_;                                         
     ros::Publisher ttg_pub_;                                           
     std::string ns_;                                                   
     std::string tracked_agents_sub_topic_;                             
   
     const HATebConfig* cfg_;     
     bool initialized_;           
     ros::Timer clearing_timer_;  
   
     bool last_publish_robot_global_plan_,          
         last_publish_robot_local_plan_,            
         last_publish_robot_local_plan_poses_,      
         last_publish_robot_local_plan_fp_poses_,   
         last_publish_agents_global_plans_,         
         last_publish_agents_local_plans_,          
         last_publish_agents_local_plan_poses_,     
         last_publish_agents_local_plan_fp_poses_;  
   
     mutable int last_robot_fp_poses_idx_,  
         last_agent_fp_poses_idx_;          
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   using TebVisualizationPtr = boost::shared_ptr<TebVisualization>;
   
   using TebVisualizationConstPtr = boost::shared_ptr<const TebVisualization>;
   
   }  // namespace hateb_local_planner
   
   // Include template method implementations / definitions
   #include <hateb_local_planner/visualization.hpp>
   
   #endif /* VISUALIZATION_H_ */
