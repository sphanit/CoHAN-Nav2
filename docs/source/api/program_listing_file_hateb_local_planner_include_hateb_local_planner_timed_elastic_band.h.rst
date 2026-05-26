
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_timed_elastic_band.h:

Program Listing for File timed_elastic_band.h
=============================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_timed_elastic_band.h>` (``hateb_local_planner/include/hateb_local_planner/timed_elastic_band.h``)

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
   
   #include <geometry_msgs/PoseArray.h>
   #include <geometry_msgs/PoseStamped.h>
   #include <hateb_local_planner/obstacles.h>
   #include <ros/assert.h>
   #include <ros/ros.h>
   #include <tf/tf.h>
   
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
       ROS_ASSERT(index < sizeTimeDiffs());
       return timediff_vec_.at(index)->dt();
     }
   
     const double& TimeDiff(int index) const {
       ROS_ASSERT(index < sizeTimeDiffs());
       return timediff_vec_.at(index)->dt();
     }
   
     PoseSE2& Pose(int index) {
       ROS_ASSERT(index < sizePoses());
       return pose_vec_.at(index)->pose();
     }
   
     const PoseSE2& Pose(int index) const {
       ROS_ASSERT(index < sizePoses());
       return pose_vec_.at(index)->pose();
     }
   
     PoseSE2& BackPose() { return pose_vec_.back()->pose(); }
   
     const PoseSE2& BackPose() const { return pose_vec_.back()->pose(); }
   
     double& BackTimeDiff() { return timediff_vec_.back()->dt(); }
   
     const double& BackTimeDiff() const { return timediff_vec_.back()->dt(); }
   
     VertexPose* PoseVertex(int index) {
       ROS_ASSERT(index < sizePoses());
       return pose_vec_.at(index);
     }
   
     VertexTimeDiff* TimeDiffVertex(int index) {
       ROS_ASSERT(index < sizeTimeDiffs());
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
   
     bool initTrajectoryToGoal(const std::vector<geometry_msgs::PoseStamped>& plan, double max_vel_x, double max_vel_theta, bool estimate_orient = false, int min_samples = 3,
                               bool guess_backwards_motion = false, double skip_dist = 0.0);
   
     ROS_DEPRECATED bool initTEBtoGoal(const PoseSE2& start, const PoseSE2& goal, double diststep = 0, double timestep = 1, int min_samples = 3, bool guess_backwards_motion = false) {
       ROS_WARN_ONCE(
           "initTEBtoGoal is deprecated and has been replaced by initTrajectoryToGoal. The signature has changed: timestep has been replaced by max_vel_x. \
                      this deprecated method sets max_vel_x = 1. Please update your code.");
       return initTrajectoryToGoal(start, goal, diststep, timestep, min_samples, guess_backwards_motion);
     }
   
     template <typename BidirIter, typename Fun>
     ROS_DEPRECATED bool initTEBtoGoal(BidirIter path_start, BidirIter path_end, Fun fun_position, double max_vel_x, double max_vel_theta, boost::optional<double> max_acc_x,
                                       boost::optional<double> max_acc_theta, boost::optional<double> start_orientation, boost::optional<double> goal_orientation, int min_samples = 3,
                                       bool guess_backwards_motion = false) {
       return initTrajectoryToGoal<BidirIter, Fun>(path_start, path_end, fun_position, max_vel_x, max_vel_theta, max_acc_x, max_acc_theta, start_orientation, goal_orientation, min_samples,
                                                   guess_backwards_motion);
     }
   
     ROS_DEPRECATED bool initTEBtoGoal(const std::vector<geometry_msgs::PoseStamped>& plan, double dt, bool estimate_orient = false, int min_samples = 3, bool guess_backwards_motion = false) {
       ROS_WARN_ONCE(
           "initTEBtoGoal is deprecated and has been replaced by initTrajectoryToGoal. The signature has changed: dt has been replaced by max_vel_x. \
                      this deprecated method sets max_vel = 1. Please update your code.");
       return initTrajectoryToGoal(plan, 1.0, 1.0, estimate_orient, min_samples, guess_backwards_motion);
     }
   
   
   
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
   
   }  // namespace hateb_local_planner
   
   // include template implementations / definitions
   #include <hateb_local_planner/timed_elastic_band.hpp>
   
   #endif /* TIMED_ELASTIC_BAND_H_ */
