
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_backoff.h:

Program Listing for File backoff.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_backoff.h>` (``hateb_local_planner/include/hateb_local_planner/backoff.h``)

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
    * Author: Phani Teja Singamaneni (email:ptsingaman@laas.fr)
    *********************************************************************/
   #ifndef BACKOFF_H_
   #define BACKOFF_H_
   
   #include <actionlib/client/simple_action_client.h>
   #include <base_local_planner/costmap_model.h>
   #include <costmap_2d/costmap_2d_ros.h>
   #include <dynamic_reconfigure/server.h>
   #include <geometry_msgs/Pose2D.h>
   #include <nav_msgs/GetPlan.h>
   #include <ros/ros.h>
   #include <std_srvs/SetBool.h>
   #include <std_srvs/Trigger.h>
   #include <tf/transform_listener.h>
   #include <tf2/utils.h>
   
   namespace hateb_local_planner {
   
   class Backoff {
    public:
     Backoff(costmap_2d::Costmap2DROS* costmap_ros);
   
     ~Backoff();
   
     void initialize(costmap_2d::Costmap2DROS* costmap_ros);
   
     bool setbackGoal(geometry_msgs::PoseStamped goal);
   
     bool timeOut();
   
     bool startRecovery();
   
     bool checkNewGoal() const { return new_goal_; }
   
     bool isBackoffGoalReached(geometry_msgs::Pose2D& robot_pose) const;
   
     void initializeOffsets(double r) {
       robot_circumscribed_radius_ = r;
       const double right_grid_offsets[4][2] = {{r, r}, {r, -r}, {-r, -r}, {-r, r}};  //{{r, -r}, {r, -3 * r}, {-r, -3 * r}, {-r, -r}};
       const double left_grid_offsets[4][2] = {{r, r}, {r, -r}, {-r, -r}, {-r, r}};   //{{r, 3 * r}, {r, r}, {-r, r}, {-r, 3 * r}};
   
       for (const auto* offset : right_grid_offsets) {
         geometry_msgs::Point p;
         p.x = offset[0];
         p.y = offset[1];
         p.z = 0;
         right_grid_.push_back(p);
       }
   
       for (const auto* offset : left_grid_offsets) {
         geometry_msgs::Point p;
         p.x = offset[0];
         p.y = offset[1];
         p.z = 0;
         left_grid_.push_back(p);
       }
     }
   
    private:
     void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
   
     static double normalize_angle(double angle_radians) {
       // Use ceres::floor because it is specialized for double and Jet types.
       double two_pi = 2.0 * M_PI;
       return angle_radians - (two_pi * std::floor((angle_radians + (M_PI)) / two_pi));
     }
   
     void loadRosParamFromNodeHandle(const ros::NodeHandle& private_nh);
   
     std::string map_frame_;           
     std::string footprint_frame_;     
     std::string ns_;                  
     std::string publish_goal_topic_;  
     std::string current_goal_topic_;  
     std::string get_plan_srv_name_;   
     double backoff_timeout_;          
   
     geometry_msgs::PoseStamped goal_;      
     geometry_msgs::PoseStamped old_goal_;  
   
     tf::TransformListener tf_;  
   
     costmap_2d::Costmap2DROS* costmap_ros_;  
     costmap_2d::Costmap2D* costmap_;         
     double robot_circumscribed_radius_;      
   
     // ROS communication members
     ros::Publisher goal_pub_;             
     ros::Publisher poly_pub_l_;           
     ros::Publisher poly_pub_r_;           
     ros::Subscriber goal_sub_;            
     ros::ServiceClient get_plan_client_;  
   
     geometry_msgs::Transform start_pose_;  
     bool visualize_backoff_;               
     bool self_published_;                  
     bool new_goal_;                        
   
     ros::Time last_time_;  
     // ros::Time last_rot_time_;   //!< Time of the last rotation movement
     // ros::Time last_goal_time_;  //!< Time when the last goal was received
   
     tf::Transform start_pose_tr_;           
     tf::StampedTransform robot_to_map_tf_;  
   
     std::shared_ptr<base_local_planner::CostmapModel> costmap_model_;  
     geometry_msgs::PoseStamped backoff_goal_;                          
   
     std::vector<geometry_msgs::Point> left_grid_;   
     std::vector<geometry_msgs::Point> right_grid_;  
   };
   
   }  // namespace hateb_local_planner
   #endif  // BACKOFF_H_
