
.. _program_listing_file_hateb_local_planner_src_backoff.cpp:

Program Listing for File backoff.cpp
====================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_src_backoff.cpp>` (``hateb_local_planner/src/backoff.cpp``)

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
   
   #include <hateb_local_planner/backoff.h>
   
   #include "ros/time.h"
   
   #define NODE_NAME "BackoffRecovery"
   // Configuarable parameters
   #define MAP_FRAME "map"
   #define FOOTPRINT_FRAME "base_footprint"
   #define PUBLISH_GOAL_TOPIC "/move_base_simple/goal"
   #define GET_PLAN_SRV_NAME "/move_base/GlobalPlanner/make_plan"
   #define CURRENT_GOAL_TOPIC_NAME "/move_base/current_goal"
   
   namespace hateb_local_planner {
   // empty constructor and destructor
   Backoff::Backoff(costmap_2d::Costmap2DROS *costmap_ros) {
     // Initialize the backoff recovery behavior
     initialize(costmap_ros);
   }
   
   Backoff::~Backoff() = default;
   
   void Backoff::initialize(costmap_2d::Costmap2DROS *costmap_ros) {
     // get private node handle
     ros::NodeHandle nh("~");
   
     // Load the parameters from the ros handle
     loadRosParamFromNodeHandle(nh);
   
     // If a namespace is associated update some topics accordingly
     if (!ns_.empty()) {
       footprint_frame_ = ns_ + "/" + footprint_frame_;
       current_goal_topic_ = "/" + ns_ + current_goal_topic_;
       publish_goal_topic_ = "/" + ns_ + publish_goal_topic_;
       get_plan_srv_name_ = "/" + ns_ + get_plan_srv_name_;
     }
   
     // Get costmap
     costmap_ros_ = costmap_ros;
     costmap_ = costmap_ros_->getCostmap();
     costmap_model_ = std::make_shared<base_local_planner::CostmapModel>(*costmap_);
   
     // Initialize ros topics, services and subscribers
     goal_sub_ = nh.subscribe(current_goal_topic_, 1, &Backoff::goalCB, this);
     goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(publish_goal_topic_, 1);
     get_plan_client_ = nh.serviceClient<nav_msgs::GetPlan>(get_plan_srv_name_, true);
     poly_pub_l_ = nh.advertise<geometry_msgs::PolygonStamped>("left_polygon", 100);
     poly_pub_r_ = nh.advertise<geometry_msgs::PolygonStamped>("right_polygon", 100);
   
     // Initialize the variables
     self_published_ = false;
     new_goal_ = false;
     last_time_ = ros::Time::now();
   
     ROS_DEBUG_NAMED(NODE_NAME, "node %s initialized", NODE_NAME);
   }
   
   void Backoff::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal) {
     // Skip if a new goal is published by this
     if (self_published_) {
       self_published_ = false;
       new_goal_ = false;
       return;
     }
     new_goal_ = true;
   }
   
   bool Backoff::startRecovery() {
     bool transform_found = false;
     auto r = robot_circumscribed_radius_;
   
     // Get the transform from robot frame to map frame
     try {
       tf_.lookupTransform(map_frame_, footprint_frame_, ros::Time(0), robot_to_map_tf_);
       transform_found = true;
     } catch (tf::LookupException &ex) {
       ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n", ex.what());
     } catch (tf::ConnectivityException &ex) {
       ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
     } catch (tf::ExtrapolationException &ex) {
       ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
     }
   
     geometry_msgs::PoseStamped backoff_goal;
     backoff_goal.header.frame_id = "map";
     backoff_goal.header.stamp = ros::Time::now();
     backoff_goal.pose.orientation.w = 1;
   
     if (transform_found && get_plan_client_) {
       std::vector<geometry_msgs::Point32> right_grid_vis;
       std::vector<geometry_msgs::Point32> left_grid_vis;
   
       // Set the polygons for searching (we assume square)
       const double right_grid_offsets_vis[4][2] = {{r, -r}, {r, -3 * r}, {-r, -3 * r}, {-r, -r}};
       const double left_grid_offsets_vis[4][2] = {{r, 3 * r}, {r, r}, {-r, r}, {-r, 3 * r}};
   
       // Initialize the necessary
       double back_dist = 0;
       double robot_theta = 0;
       double search_angle = 0.0;
       double angle_increment = 0.174;
       bool found = false;
       bool flipped = false;
       int count = 0;
   
       std::vector<geometry_msgs::PoseStamped> goals;
       nav_msgs::GetPlan get_plan_srv;
       get_plan_srv.request.start.header.frame_id = "map";
       tf::poseTFToMsg(robot_to_map_tf_, get_plan_srv.request.start.pose);
   
       get_plan_srv.request.goal.header.frame_id = "map";
   
       // Start the search
       while (true) {
         start_pose_tr_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
         start_pose_tr_.setRotation(tf::createQuaternionFromYaw(search_angle));
         start_pose_tr_ = robot_to_map_tf_ * start_pose_tr_;
         tf::transformTFToMsg(start_pose_tr_, start_pose_);
         robot_theta = tf2::getYaw(start_pose_.rotation);
         // Clearing visualizes only the lastest search angle.
         // left_grid_vis.clear();
         // right_grid_vis.clear();
   
         // Search upto 50 configurations for each angle
         // TODO: Needs to configure or adjust this number
         for (int i = 0; i < 50; i++) {
           if (visualize_backoff_) {
             tf::Vector3 point_tf;
             point_tf.setZero();
   
             // Get the configuration on right
             for (const auto *offset : right_grid_offsets_vis) {
               point_tf.setValue(offset[0] - back_dist, offset[1], 0.0);
               point_tf = start_pose_tr_ * point_tf;
   
               geometry_msgs::Point32 p32;
               p32.x = point_tf.x();
               p32.y = point_tf.y();
               p32.z = 0;
               right_grid_vis.push_back(p32);
             }
   
             // Get the configuration on left
             for (const auto *offset : left_grid_offsets_vis) {
               point_tf.setValue(offset[0] - back_dist, offset[1], 0.0);
               point_tf = start_pose_tr_ * point_tf;
   
               geometry_msgs::Point32 p32;
               p32.x = point_tf.x();
               p32.y = point_tf.y();
               p32.z = 0;
               left_grid_vis.push_back(p32);
             }
   
             // Publish the Left and Right polygons
             geometry_msgs::PolygonStamped r_polygon;
             r_polygon.header.frame_id = "map";
             r_polygon.header.stamp = ros::Time::now();
             r_polygon.polygon.points = right_grid_vis;
             poly_pub_r_.publish(r_polygon);
   
             geometry_msgs::PolygonStamped l_polygon;
             l_polygon.header.frame_id = "map";
             l_polygon.header.stamp = ros::Time::now();
             l_polygon.polygon.points = left_grid_vis;
             poly_pub_l_.publish(l_polygon);
           }
   
           // Find the polygon center on right
           auto r_center = tf::Vector3(-back_dist, -2 * r, 0.0);
           r_center = start_pose_tr_ * r_center;
   
           // FInd the polygon center on left
           auto l_center = tf::Vector3(-back_dist, 2 * r, 0.0);
           l_center = start_pose_tr_ * l_center;
   
           // Check overlap with costmap (right)
           if (costmap_model_->footprintCost(r_center.x(), r_center.y(), robot_theta, right_grid_) == 0) {
             // ROS_INFO("Found safe spot on right");
             backoff_goal.pose.position.x = r_center.x();
             backoff_goal.pose.position.y = r_center.y();
             backoff_goal.pose.position.z = 0;
             backoff_goal.pose.orientation = start_pose_.rotation;
             goals.push_back(backoff_goal);
             found = true;
             count++;
             break;
           }
   
           // Check overlap with costmap (left)
           if (costmap_model_->footprintCost(l_center.x(), l_center.y(), robot_theta, left_grid_) == 0) {
             // ROS_INFO("Found safe spot on left");
             backoff_goal.pose.position.x = l_center.x();
             backoff_goal.pose.position.y = l_center.y();
             backoff_goal.pose.position.z = 0;
             backoff_goal.pose.orientation = start_pose_.rotation;
             goals.push_back(backoff_goal);
             found = true;
             count++;
             break;
           }
   
           // Increment the distance and repeat
           back_dist += 0.5;
         }
   
         // Reset the distance for new angle
         back_dist = 0.0;
   
         // increase the angle to continue search
         if (!flipped) {
           search_angle += angle_increment;
           if (search_angle > M_PI / 2) {
             // Flip when positive limit is reached
             search_angle = -0.175;
             flipped = true;
           }
         }
   
         else {
           search_angle -= angle_increment;
           // Break when negative limit is reached
           if (search_angle < -M_PI / 2) {
             break;
           }
         }
       }
       if (!found) {
         ROS_INFO("No safespot found with the current search range!");
         return false;
       }
   
       std::map<int, int> length_idx_map;
   
       // Interate through the positions and get the length of global plans.
       // The global plan lengths are then used to determine the backoff goal.
       for (int i = 0; i < count; i++) {
         get_plan_srv.request.start.header.stamp = ros::Time::now();
         get_plan_srv.request.goal.header.stamp = ros::Time::now();
   
         get_plan_srv.request.goal.pose.position.x = goals[i].pose.position.x;
         get_plan_srv.request.goal.pose.position.y = goals[i].pose.position.y;
         get_plan_srv.request.goal.pose.position.z = 0;
         get_plan_srv.request.goal.pose.orientation = goals[i].pose.orientation;
         get_plan_client_.call(get_plan_srv);
   
         if (!get_plan_srv.response.plan.poses.empty()) {
           length_idx_map[get_plan_srv.response.plan.poses.size()] = i;
         }
       }
       // Map arranges the distances sorted according to length.
       // Select the one with shortest length for backoff.
       backoff_goal_ = goals[length_idx_map.begin()->second];
     }
   
     // Finally publish the backoff goal
     self_published_ = true;
     start_pose_tr_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
     start_pose_tr_.setRotation(tf::createQuaternionFromYaw(0.0));
     start_pose_tr_ = robot_to_map_tf_ * start_pose_tr_;
     tf::transformTFToMsg(start_pose_tr_, start_pose_);
   
     goal_pub_.publish(backoff_goal_);
     last_time_ = ros::Time::now();
   
     return true;
   }
   
   bool Backoff::setbackGoal(geometry_msgs::PoseStamped goal) {
     // Set back the goal that is existing before recovery
     ROS_INFO("Setting back the goal !!");
     goal.header.stamp = ros::Time::now();
     goal.header.frame_id = "map";
     goal_pub_.publish(goal);
     ROS_INFO("Goal set back success");
     return true;
   }
   
   bool Backoff::timeOut() {
     // Return TRUE after a given timeout
     return (ros::Time::now() - last_time_).toSec() > backoff_timeout_;
   }
   
   bool Backoff::isBackoffGoalReached(geometry_msgs::Pose2D &robot_pose) const {
     // True if Backoff goal is reached
     double delta_orient = normalize_angle(tf2::getYaw(backoff_goal_.pose.orientation) - robot_pose.theta);
     double dg = std::hypot(backoff_goal_.pose.position.x - robot_pose.x, backoff_goal_.pose.position.y - robot_pose.y);
   
     return fabs(dg) < 0.2 && fabs(delta_orient) < 0.2;
   }
   
   void Backoff::loadRosParamFromNodeHandle(const ros::NodeHandle &private_nh) {
     private_nh.param("ns", ns_, std::string(""));
     private_nh.param("get_plan_srv_name", get_plan_srv_name_, std::string(GET_PLAN_SRV_NAME));
     private_nh.param("current_goal_topic", current_goal_topic_, std::string(CURRENT_GOAL_TOPIC_NAME));
     private_nh.param("publish_goal_topic", publish_goal_topic_, std::string(PUBLISH_GOAL_TOPIC));
     private_nh.param("footprint_frame", footprint_frame_, std::string(FOOTPRINT_FRAME));
     private_nh.param("map_frame", map_frame_, std::string(MAP_FRAME));
     private_nh.param("visualize_backoff", visualize_backoff_, false);
     private_nh.param("backoff_timeout", backoff_timeout_, 30.0);
   }
   
   }  // namespace hateb_local_planner
