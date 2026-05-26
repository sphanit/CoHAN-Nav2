
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_pose_se2.h:

Program Listing for File pose_se2.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_pose_se2.h>` (``hateb_local_planner/include/hateb_local_planner/pose_se2.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    * Modified by Phani Teja Singamaneni in 2021
    * Additional changes licensed under the MIT License. See LICENSE file.
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
   
   #ifndef POSE_SE2_H_
   #define POSE_SE2_H_
   
   #include <g2o/stuff/misc.h>
   #include <hateb_local_planner/misc.h>
   #include <tf2/utils.h>
   
   #include <Eigen/Core>
   #include <geometry_msgs/msg/pose.hpp>
   #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
   
   namespace hateb_local_planner {
   
   class PoseSE2 {
    public:
   
     PoseSE2() { setZero(); }
   
     PoseSE2(const Eigen::Ref<const Eigen::Vector2d>& position, double theta) {
       position_ = position;
       theta_ = theta;
     }
   
     PoseSE2(double x, double y, double theta) {
       position_.coeffRef(0) = x;
       position_.coeffRef(1) = y;
       theta_ = theta;
     }
   
     explicit PoseSE2(const geometry_msgs::msg::Pose& pose) {
       position_.coeffRef(0) = pose.position.x;
       position_.coeffRef(1) = pose.position.y;
       theta_ = tf2::getYaw(pose.orientation);
     }
   
     PoseSE2(const PoseSE2& pose) {
       position_ = pose.position_;
       theta_ = pose.theta_;
     }
   
   
     ~PoseSE2() = default;
   
   
     Eigen::Vector2d& position() { return position_; }
   
     const Eigen::Vector2d& position() const { return position_; }
   
     double& x() { return position_.coeffRef(0); }
   
     const double& x() const { return position_.coeffRef(0); }
   
     double& y() { return position_.coeffRef(1); }
   
     const double& y() const { return position_.coeffRef(1); }
   
     double& theta() { return theta_; }
   
     const double& theta() const { return theta_; }
   
     void setZero() {
       position_.setZero();
       theta_ = 0;
     }
   
     void toPoseMsg(geometry_msgs::msg::Pose& pose) const {
       pose.position.x = position_.x();
       pose.position.y = position_.y();
       pose.position.z = 0;
       tf2::Quaternion q;
       q.setRPY(0, 0, theta_);
       pose.orientation = tf2::toMsg(q);
     }
   
     Eigen::Vector2d orientationUnitVec() const { return Eigen::Vector2d(std::cos(theta_), std::sin(theta_)); }
   
   
   
     void scale(double factor) {
       position_ *= factor;
       theta_ = g2o::normalize_theta(theta_ * factor);
     }
   
     void plus(const double* pose_as_array) {
       position_.coeffRef(0) += pose_as_array[0];
       position_.coeffRef(1) += pose_as_array[1];
       theta_ = g2o::normalize_theta(theta_ + pose_as_array[2]);
     }
   
     void averageInPlace(const PoseSE2& pose1, const PoseSE2& pose2) {
       position_ = (pose1.position_ + pose2.position_) / 2;
       theta_ = g2o::average_angle(pose1.theta_, pose2.theta_);
     }
   
     static PoseSE2 average(const PoseSE2& pose1, const PoseSE2& pose2) { return PoseSE2((pose1.position_ + pose2.position_) / 2, g2o::average_angle(pose1.theta_, pose2.theta_)); }
   
     void rotateGlobal(double angle, bool adjust_theta = true) {
       double new_x = (std::cos(angle) * position_.x()) - (std::sin(angle) * position_.y());
       double new_y = (std::sin(angle) * position_.x()) + (std::cos(angle) * position_.y());
       position_.x() = new_x;
       position_.y() = new_y;
       if (adjust_theta) theta_ = g2o::normalize_theta(theta_ + angle);
     }
   
   
   
     PoseSE2& operator=(const PoseSE2& rhs) {
       if (&rhs != this) {
         position_ = rhs.position_;
         theta_ = rhs.theta_;
       }
       return *this;
     }
   
     PoseSE2& operator+=(const PoseSE2& rhs) {
       position_ += rhs.position_;
       theta_ = g2o::normalize_theta(theta_ + rhs.theta_);
       return *this;
     }
   
     friend PoseSE2 operator+(PoseSE2 lhs, const PoseSE2& rhs) { return lhs += rhs; }
   
     PoseSE2& operator-=(const PoseSE2& rhs) {
       position_ -= rhs.position_;
       theta_ = g2o::normalize_theta(theta_ - rhs.theta_);
       return *this;
     }
   
     friend PoseSE2 operator-(PoseSE2 lhs, const PoseSE2& rhs) { return lhs -= rhs; }
   
     friend PoseSE2 operator*(PoseSE2 pose, double scalar) {
       pose.position_ *= scalar;
       pose.theta_ *= scalar;
       return pose;
     }
   
     friend PoseSE2 operator*(double scalar, PoseSE2 pose) {
       pose.position_ *= scalar;
       pose.theta_ *= scalar;
       return pose;
     }
   
     friend std::ostream& operator<<(std::ostream& stream, const PoseSE2& pose) {
       stream << "x: " << pose.position_[0] << " y: " << pose.position_[1] << " theta: " << pose.theta_;
       return stream;
     }
   
   
    private:
     Eigen::Vector2d position_;
     double theta_;
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   }  // namespace hateb_local_planner
   
   #endif  // POSE_SE2_H_
