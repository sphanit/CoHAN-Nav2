
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_timed_elastic_band.hpp:

Program Listing for File timed_elastic_band.hpp
===============================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_timed_elastic_band.hpp>` (``hateb_local_planner/include/hateb_local_planner/timed_elastic_band.hpp``)

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
    *********************************************************************/
   
   #ifndef TIMED_ELASTIC_BAND_H_
   #define TIMED_ELASTIC_BAND_H_
   
   #include <hateb_local_planner/obstacles.h>
   #include <tf2/utils.h>
   
   #include <boost/optional.hpp>
   #include <geometry_msgs/msg/pose_array.hpp>
   #include <geometry_msgs/msg/pose_stamped.hpp>
   #include <limits>
   #include <rclcpp/logging.hpp>
   #include <rclcpp/rclcpp.hpp>
   #include <ros2_helpers/utils.hpp>
   
   // G2O Types
   #include <hateb_local_planner/g2o_types/vertex_pose.h>
   #include <hateb_local_planner/g2o_types/vertex_timediff.h>
   
   namespace hateb_local_planner {
   
   using PoseSequence = std::vector<VertexPose*>;
   using TimeDiffSequence = std::vector<VertexTimeDiff*>;
   
   class TimedElasticBand {
    public:
     TimedElasticBand();
   
     virtual ~TimedElasticBand();
   
   
     PoseSequence& poses() { return pose_vec_; };
   
     const PoseSequence& poses() const { return pose_vec_; };
   
     TimeDiffSequence& timediffs() { return timediff_vec_; };
   
     const TimeDiffSequence& timediffs() const { return timediff_vec_; };
   
     double& TimeDiff(int index) {
       RCLCPP_ASSERT(index < sizeTimeDiffs());
       return timediff_vec_.at(index)->dt();
     }
   
     const double& TimeDiff(int index) const {
       RCLCPP_ASSERT(index < sizeTimeDiffs());
       return timediff_vec_.at(index)->dt();
     }
   
     PoseSE2& Pose(int index) {
       RCLCPP_ASSERT(index < sizePoses());
       return pose_vec_.at(index)->pose();
     }
   
     const PoseSE2& Pose(int index) const {
       RCLCPP_ASSERT(index < sizePoses());
       return pose_vec_.at(index)->pose();
     }
   
     PoseSE2& BackPose() { return pose_vec_.back()->pose(); }
   
     const PoseSE2& BackPose() const { return pose_vec_.back()->pose(); }
   
     double& BackTimeDiff() { return timediff_vec_.back()->dt(); }
   
     const double& BackTimeDiff() const { return timediff_vec_.back()->dt(); }
   
     VertexPose* PoseVertex(int index) {
       RCLCPP_ASSERT(index < sizePoses());
       return pose_vec_.at(index);
     }
   
     VertexTimeDiff* TimeDiffVertex(int index) {
       RCLCPP_ASSERT(index < sizeTimeDiffs());
       return timediff_vec_.at(index);
     }
   
   
   
     void addPose(const PoseSE2& pose, bool fixed = false);
   
     void addPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed = false);
   
     void addPose(double x, double y, double theta, bool fixed = false);
   
     void addTimeDiff(double dt, bool fixed = false);
   
     void addPoseAndTimeDiff(const PoseSE2& pose, double dt);
   
     void addPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, double dt);
   
     void addPoseAndTimeDiff(double x, double y, double theta, double dt);
   
   
   
     void insertPose(int index, const PoseSE2& pose);
   
     void insertPose(int index, const Eigen::Ref<const Eigen::Vector2d>& position, double theta);
   
     void insertPose(int index, double x, double y, double theta);
   
     void insertTimeDiff(int index, double dt);
   
     void deletePose(int index);
   
     void deletePoses(int index, int number);
   
     void deleteTimeDiff(int index);
   
     void deleteTimeDiffs(int index, int number);
   
   
   
     bool initTrajectoryToGoal(const PoseSE2& start, const PoseSE2& goal, double diststep = 0, double max_vel_x = 0.5, int min_samples = 3, bool guess_backwards_motion = false);
   
     template <typename BidirIter, typename Fun>
     bool initTrajectoryToGoal(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta, boost::optional<double> max_acc_x,
                               boost::optional<double> max_acc_theta, boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples = 3,
                               bool guess_backwards_motion = false);
   
     bool initTrajectoryToGoal(const std::vector<geometry_msgs::msg::PoseStamped>& plan, double max_vel_x, double max_vel_theta, bool estimate_orient = false, int min_samples = 3,
                               bool guess_backwards_motion = false, double skip_dist = 0.0);
   
     void updateAndPruneTEB(boost::optional<const PoseSE2&> new_start, boost::optional<const PoseSE2&> new_goal, int min_samples = 3);
   
     void autoResize(double dt_ref, double dt_hysteresis, int min_samples = 3, int max_samples = 1000, bool fast_mode = false);
   
     void setPoseVertexFixed(int index, bool status);
   
     void setTimeDiffVertexFixed(int index, bool status);
   
     void clearTimedElasticBand();
   
   
   
     int findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_point, double* distance = nullptr, int begin_idx = 0) const;
   
     int findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_line_start, const Eigen::Ref<const Eigen::Vector2d>& ref_line_end, double* distance = nullptr) const;
   
     int findClosestTrajectoryPose(const Point2dContainer& vertices, double* distance = nullptr) const;
   
     int findClosestTrajectoryPose(const Obstacle& obstacle, double* distance = nullptr) const;
   
     int sizePoses() const { return static_cast<int>(pose_vec_.size()); };
   
     int sizeTimeDiffs() const { return static_cast<int>(timediff_vec_.size()); };
   
     bool isInit() const { return !timediff_vec_.empty() && !pose_vec_.empty(); }
   
     double getSumOfAllTimeDiffs() const;
   
     double getSumOfTimeDiffsUpToIdx(int index) const;
   
     double getAccumulatedDistance() const;
   
     bool isTrajectoryInsideRegion(double radius, double max_dist_behind_robot = -1, int skip_poses = 0);
   
   
    protected:
     PoseSequence pose_vec_;          
     TimeDiffSequence timediff_vec_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   // Template implementation
   template <typename BidirIter, typename Fun>
   bool TimedElasticBand::initTrajectoryToGoal(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta, boost::optional<double> max_acc_x,
                                               boost::optional<double> max_acc_theta, boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples,
                                               bool guess_backwards_motion) {
     Eigen::Vector2d start_position = fun_position(*path_start);
     Eigen::Vector2d goal_position = fun_position(*std::prev(path_end));
   
     bool backwards = false;
   
     double start_orient;
     double goal_orient;
     if (start_orientation) {
       start_orient = *start_orientation;
   
       // check if the goal is behind the start pose (w.r.t. start orientation)
       if (guess_backwards_motion && (goal_position - start_position).dot(Eigen::Vector2d(std::cos(start_orient), std::sin(start_orient))) < 0) {
         backwards = true;
       }
     } else {
       Eigen::Vector2d start2goal = goal_position - start_position;
       start_orient = atan2(start2goal[1], start2goal[0]);
     }
   
     double timestep = 1;  // TODO: time
   
     if (goal_orientation) {
       goal_orient = *goal_orientation;
     } else {
       goal_orient = start_orient;
     }
   
     if (!isInit()) {
       addPose(start_position, start_orient, true);  // add starting point and mark it as fixed for optimization
   
       // we insert middle points now (increase start by 1 and decrease goal by 1)
       std::advance(path_start, 1);
       std::advance(path_end, -1);
       int idx = 0;
       for (; path_start != path_end; ++path_start)  // insert middle-points
       {
         // Eigen::Vector2d point_to_goal = path.back()-*it;
         // double dir_to_goal = atan2(point_to_goal[1],point_to_goal[0]); // direction to goal
         // Alternative: Direction from last path
         Eigen::Vector2d curr_point = fun_position(*path_start);
         Eigen::Vector2d diff_last = curr_point - Pose(idx).position();  // we do not use boost::prior(*path_start) for those cases,
                                                                         // where fun_position() does not return a reference or is expensive.
         double diff_norm = diff_last.norm();
   
         double timestep_vel = diff_norm / max_vel_x;  // constant velocity
         double timestep_acc;
   
         if (max_acc_x) {
           timestep_acc = sqrt(2 * diff_norm / (*max_acc_x));  // constant acceleration
           if (timestep_vel < timestep_acc && max_acc_x) {
             timestep = timestep_acc;
           } else {
             timestep = timestep_vel;
           }
         } else {
           timestep = timestep_vel;
         }
   
         if (timestep <= 0) {
           timestep = 0.2;  // TODO: this is an assumption
         }
   
         double yaw = atan2(diff_last[1], diff_last[0]);
         if (backwards) {
           yaw = g2o::normalize_theta(yaw + M_PI);
         }
         addPoseAndTimeDiff(curr_point, yaw, timestep);
   
         ++idx;
       }
       Eigen::Vector2d diff = goal_position - Pose(idx).position();
       double diff_norm = diff.norm();
       double timestep_vel = diff_norm / max_vel_x;  // constant velocity
       if (max_acc_x) {
         double timestep_acc = sqrt(2 * diff_norm / (*max_acc_x));  // constant acceleration
         if (timestep_vel < timestep_acc) {
           timestep = timestep_acc;
         } else {
           timestep = timestep_vel;
         }
       } else {
         timestep = timestep_vel;
       }
   
       PoseSE2 goal(goal_position, goal_orient);
   
       // if number of samples is not larger than min_samples, insert manually
       if (sizePoses() < min_samples - 1) {
         RCLCPP_DEBUG(rclcpp::get_logger("TimedElasticBand"), "initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
         while (sizePoses() < min_samples - 1)  // subtract goal point that will be added later
         {
           // Each inserted point bisects the remaining distance. Thus the timestep is also bisected.
           timestep /= 2;
           // simple strategy: interpolate between the current pose and the goal
           addPoseAndTimeDiff(PoseSE2::average(BackPose(), goal), timestep);  // let the optimier correct the timestep (TODO: better initialization
         }
       }
   
       // now add goal
       addPoseAndTimeDiff(goal, timestep);         // add goal point
       setPoseVertexFixed(sizePoses() - 1, true);  // GoalConf is a fixed constraint during optimization
     } else                                        // size!=0
     {
       RCLCPP_WARN(rclcpp::get_logger("TimedElasticBand"),
                   "Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
       RCLCPP_WARN(rclcpp::get_logger("TimedElasticBand"), "Number of TEB configurations: %d, Number of TEB timediffs: %d", sizePoses(), sizeTimeDiffs());
       return false;
     }
     return true;
   }
   
   }  // namespace hateb_local_planner
   
   #endif /* TIMED_ELASTIC_BAND_H_ */
