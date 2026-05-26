
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_obstacles.h:

Program Listing for File obstacles.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_obstacles.h>` (``hateb_local_planner/include/hateb_local_planner/obstacles.h``)

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
   
   #ifndef OBSTACLES_H
   #define OBSTACLES_H
   
   #include <hateb_local_planner/distance_calculations.h>
   
   #include <Eigen/Core>
   #include <Eigen/Geometry>
   #include <Eigen/StdVector>
   #include <boost/pointer_cast.hpp>
   #include <boost/shared_ptr.hpp>
   #include <complex>
   #include <geometry_msgs/msg/polygon.hpp>
   #include <geometry_msgs/msg/quaternion_stamped.hpp>
   #include <geometry_msgs/msg/twist_with_covariance.hpp>
   #include <utility>
   
   namespace hateb_local_planner {
   
   class Obstacle {
    public:
     Obstacle() : centroid_velocity_(Eigen::Vector2d::Zero()) {}
   
     virtual ~Obstacle() = default;
   
   
     virtual const Eigen::Vector2d& getCentroid() const = 0;
   
     virtual std::complex<double> getCentroidCplx() const = 0;
   
   
   
     virtual bool checkCollision(const Eigen::Vector2d& position, double min_dist) const = 0;
   
     virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const = 0;
   
     virtual double getMinimumDistance(const Eigen::Vector2d& position) const = 0;
   
     virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const = 0;
   
     virtual double getMinimumDistance(const Point2dContainer& polygon) const = 0;
   
     virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const = 0;
   
   
   
     virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const = 0;
   
     virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const = 0;
   
     virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const = 0;
   
     virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const { position = getCentroid() + t * getCentroidVelocity(); }
   
     bool isDynamic() const { return dynamic_; }
   
     bool isHuman() const { return human_; }
   
     void setHuman() { human_ = true; }
   
     void setCentroidVelocity(const Eigen::Ref<const Eigen::Vector2d>& vel) {
       centroid_velocity_ = vel;
       dynamic_ = true;
     }
   
     void setCentroidVelocity(const geometry_msgs::msg::TwistWithCovariance& velocity, const geometry_msgs::msg::Quaternion& orientation) {
       // Set velocity, if obstacle is moving
       Eigen::Vector2d vel;
       vel.coeffRef(0) = velocity.twist.linear.x;
       vel.coeffRef(1) = velocity.twist.linear.y;
   
       // If norm of velocity is less than 0.001, consider obstacle as not dynamic
       if (vel.norm() < 1e-3) {
         return;
       }
       setCentroidVelocity(vel);
     }
   
     void setCentroidVelocity(const geometry_msgs::msg::TwistWithCovariance& velocity, const geometry_msgs::msg::QuaternionStamped& orientation) { setCentroidVelocity(velocity, orientation.quaternion); }
   
     const Eigen::Vector2d& getCentroidVelocity() const { return centroid_velocity_; }
   
   
   
     virtual void toPolygonMsg(geometry_msgs::msg::Polygon& polygon) = 0;
   
     virtual void toTwistWithCovarianceMsg(geometry_msgs::msg::TwistWithCovariance& twistWithCovariance) {
       if (dynamic_) {
         twistWithCovariance.twist.linear.x = centroid_velocity_(0);
         twistWithCovariance.twist.linear.y = centroid_velocity_(1);
       } else {
         twistWithCovariance.twist.linear.x = 0;
         twistWithCovariance.twist.linear.y = 0;
       }
     }
   
   
    protected:
     bool dynamic_{};  
     bool human_{};
     Eigen::Vector2d centroid_velocity_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   using ObstaclePtr = boost::shared_ptr<Obstacle>;
   using ObstacleConstPtr = boost::shared_ptr<const Obstacle>;
   using ObstContainer = std::vector<ObstaclePtr>;
   
   class PointObstacle : public Obstacle {
    public:
     PointObstacle() : pos_(Eigen::Vector2d::Zero()) {}
   
     explicit PointObstacle(const Eigen::Ref<const Eigen::Vector2d>& position) : pos_(position) {}
   
     PointObstacle(double x, double y) : pos_(Eigen::Vector2d(x, y)) {}
   
     // implements checkCollision() of the base class
     bool checkCollision(const Eigen::Vector2d& point, double min_dist) const override { return getMinimumDistance(point) < min_dist; }
   
     // implements checkLineIntersection() of the base class
     bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const override {
       // Distance Line - Circle
       // refer to http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
       Eigen::Vector2d a = line_end - line_start;  // not normalized!  a=y-x
       Eigen::Vector2d b = pos_ - line_start;      // b=m-x
   
       // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to 0<=t<=1
       double t = a.dot(b) / a.dot(a);
       if (t < 0) {
         t = 0;
       }  // bound t (since a is not normalized, t can be scaled between 0 and 1 to parametrize the line
       else if (t > 1) {
         t = 1;
       }
       Eigen::Vector2d nearest_point = line_start + a * t;
   
       // check collision
       return checkCollision(nearest_point, min_dist);
     }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Eigen::Vector2d& position) const override { return (position - pos_).norm(); }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const override { return distance_point_to_segment_2d(pos_, line_start, line_end); }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Point2dContainer& polygon) const override { return distance_point_to_polygon_2d(pos_, polygon); }
   
     // implements getMinimumDistanceVec() of the base class
     Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const override { return pos_; }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const override { return (pos_ + t * centroid_velocity_ - position).norm(); }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const override {
       return distance_point_to_segment_2d(pos_ + t * centroid_velocity_, line_start, line_end);
     }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const override { return distance_point_to_polygon_2d(pos_ + t * centroid_velocity_, polygon); }
   
     // implements predictCentroidConstantVelocity() of the base class
     void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const override { position = pos_ + t * centroid_velocity_; }
   
     // implements getCentroid() of the base class
     const Eigen::Vector2d& getCentroid() const override { return pos_; }
   
     // implements getCentroidCplx() of the base class
     std::complex<double> getCentroidCplx() const override { return std::complex<double>(pos_[0], pos_[1]); }
   
     void setCentroid(double x, double y) {
       pos_[0] = x;
       pos_[1] = y;
     }
   
     // Accessor methods
     const Eigen::Vector2d& position() const { return pos_; }  
     Eigen::Vector2d& position() { return pos_; }              
     double& x() { return pos_.coeffRef(0); }                  
     const double& x() const { return pos_.coeffRef(0); }      
     double& y() { return pos_.coeffRef(1); }                  
     const double& y() const { return pos_.coeffRef(1); }      
   
     // implements toPolygonMsg() of the base class
     void toPolygonMsg(geometry_msgs::msg::Polygon& polygon) override {
       polygon.points.resize(1);
       polygon.points.front().x = pos_.x();
       polygon.points.front().y = pos_.y();
       polygon.points.front().z = 0;
     }
   
    protected:
     Eigen::Vector2d pos_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class CircularObstacle : public Obstacle {
    public:
     CircularObstacle() : pos_(Eigen::Vector2d::Zero()) {}
   
     CircularObstacle(const Eigen::Ref<const Eigen::Vector2d>& position, double radius) : pos_(position), radius_(radius) {}
   
     CircularObstacle(double x, double y, double radius) : pos_(Eigen::Vector2d(x, y)), radius_(radius) {}
   
     // implements checkCollision() of the base class
     bool checkCollision(const Eigen::Vector2d& point, double min_dist) const override { return getMinimumDistance(point) < min_dist; }
   
     // implements checkLineIntersection() of the base class
     bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const override {
       // Distance Line - Circle
       // refer to http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
       Eigen::Vector2d a = line_end - line_start;  // not normalized!  a=y-x
       Eigen::Vector2d b = pos_ - line_start;      // b=m-x
   
       // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to 0<=t<=1
       double t = a.dot(b) / a.dot(a);
       if (t < 0) {
         {
           t = 0;
         }
       }  // bound t (since a is not normalized, t can be scaled between 0 and 1 to parametrize the line
       else if (t > 1) {
         {
           t = 1;
         }
       }
       Eigen::Vector2d nearest_point = line_start + a * t;
   
       // check collision
       return checkCollision(nearest_point, min_dist);
     }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Eigen::Vector2d& position) const override { return (position - pos_).norm() - radius_; }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const override { return distance_point_to_segment_2d(pos_, line_start, line_end) - radius_; }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Point2dContainer& polygon) const override { return distance_point_to_polygon_2d(pos_, polygon) - radius_; }
   
     // implements getMinimumDistanceVec() of the base class
     Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const override { return pos_ + radius_ * (position - pos_).normalized(); }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const override { return (pos_ + t * centroid_velocity_ - position).norm() - radius_; }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const override {
       return distance_point_to_segment_2d(pos_ + t * centroid_velocity_, line_start, line_end) - radius_;
     }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const override { return distance_point_to_polygon_2d(pos_ + t * centroid_velocity_, polygon) - radius_; }
   
     // implements predictCentroidConstantVelocity() of the base class
     void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const override { position = pos_ + t * centroid_velocity_; }
   
     // implements getCentroid() of the base class
     const Eigen::Vector2d& getCentroid() const override { return pos_; }
   
     // implements getCentroidCplx() of the base class
     std::complex<double> getCentroidCplx() const override { return std::complex<double>(pos_[0], pos_[1]); }
   
     // Accessor methods
     const Eigen::Vector2d& position() const { return pos_; }  
     Eigen::Vector2d& position() { return pos_; }              
     double& x() { return pos_.coeffRef(0); }                  
     const double& x() const { return pos_.coeffRef(0); }      
     double& y() { return pos_.coeffRef(1); }                  
     const double& y() const { return pos_.coeffRef(1); }      
     double& radius() { return radius_; }                      
     const double& radius() const { return radius_; }          
   
     // implements toPolygonMsg() of the base class
     void toPolygonMsg(geometry_msgs::msg::Polygon& polygon) override {
       // TODO(roesmann): the polygon message type cannot describe a "perfect" circle
       //                 We could switch to ObstacleMsg if required somewhere...
       polygon.points.resize(1);
       polygon.points.front().x = pos_.x();
       polygon.points.front().y = pos_.y();
       polygon.points.front().z = 0;
     }
   
    protected:
     Eigen::Vector2d pos_;  
     double radius_ = 0.0;  
   
    public:
     void setCentroid(double x, double y) {
       pos_[0] = x;
       pos_[1] = y;
     }
   
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class LineObstacle : public Obstacle {
    public:
     using VertexContainer = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
   
     LineObstacle() {
       start_.setZero();
       end_.setZero();
       centroid_.setZero();
     }
   
     LineObstacle(const Eigen::Ref<const Eigen::Vector2d>& line_start, const Eigen::Ref<const Eigen::Vector2d>& line_end) : start_(line_start), end_(line_end) { calcCentroid(); }
   
     LineObstacle(double x1, double y1, double x2, double y2) {
       start_.x() = x1;
       start_.y() = y1;
       end_.x() = x2;
       end_.y() = y2;
       calcCentroid();
     }
   
     // implements checkCollision() of the base class
     bool checkCollision(const Eigen::Vector2d& point, double min_dist) const override { return getMinimumDistance(point) <= min_dist; }
   
     // implements checkLineIntersection() of the base class
     bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const override {
       return check_line_segments_intersection_2d(line_start, line_end, start_, end_);
     }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Eigen::Vector2d& position) const override { return distance_point_to_segment_2d(position, start_, end_); }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const override { return distance_segment_to_segment_2d(start_, end_, line_start, line_end); }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Point2dContainer& polygon) const override { return distance_segment_to_polygon_2d(start_, end_, polygon); }
   
     // implements getMinimumDistanceVec() of the base class
     Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const override { return closest_point_on_line_segment_2d(position, start_, end_); }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const override {
       Eigen::Vector2d offset = t * centroid_velocity_;
       return distance_point_to_segment_2d(position, start_ + offset, end_ + offset);
     }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const override {
       Eigen::Vector2d offset = t * centroid_velocity_;
       return distance_segment_to_segment_2d(start_ + offset, end_ + offset, line_start, line_end);
     }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const override {
       Eigen::Vector2d offset = t * centroid_velocity_;
       return distance_segment_to_polygon_2d(start_ + offset, end_ + offset, polygon);
     }
   
     // implements getCentroid() of the base class
     const Eigen::Vector2d& getCentroid() const override { return centroid_; }
   
     // implements getCentroidCplx() of the base class
     std::complex<double> getCentroidCplx() const override { return std::complex<double>(centroid_.x(), centroid_.y()); }
   
     // Access or modify line
     const Eigen::Vector2d& start() const { return start_; }
     void setStart(const Eigen::Ref<const Eigen::Vector2d>& start) {
       start_ = start;
       calcCentroid();
     }
     const Eigen::Vector2d& end() const { return end_; }
     void setEnd(const Eigen::Ref<const Eigen::Vector2d>& end) {
       end_ = end;
       calcCentroid();
     }
   
     // implements toPolygonMsg() of the base class
     void toPolygonMsg(geometry_msgs::msg::Polygon& polygon) override {
       polygon.points.resize(2);
       polygon.points.front().x = start_.x();
       polygon.points.front().y = start_.y();
   
       polygon.points.back().x = end_.x();
       polygon.points.back().y = end_.y();
       polygon.points.back().z = polygon.points.front().z = 0;
     }
   
    protected:
     void calcCentroid() { centroid_ = 0.5 * (start_ + end_); }
   
    private:
     Eigen::Vector2d start_;
     Eigen::Vector2d end_;
   
     Eigen::Vector2d centroid_;
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class PolygonObstacle : public Obstacle {
    public:
     PolygonObstacle() : finalized_(false) { centroid_.setConstant(NAN); }
   
     explicit PolygonObstacle(Point2dContainer vertices) : vertices_(std::move(vertices)) { finalizePolygon(); }
   
     /* FIXME Not working at the moment due to the aligned allocator version of std::vector
       * And it is C++11 code that is disabled atm to ensure compliance with ROS indigo/jade
   ... Vector2dType>
     PolygonObstacle(const Vector2dType&... vertices) : _vertices({vertices...})
     {
       calcCentroid();
       _finalized = true;
     }
     */
   
     // implements checkCollision() of the base class
     bool checkCollision(const Eigen::Vector2d& point, double min_dist) const override {
       // line case
       if (noVertices() == 2) {
         return getMinimumDistance(point) <= min_dist;
       }
   
       // check if point is in the interior of the polygon
       // point in polygon test - raycasting (http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html)
       // using the following algorithm we may obtain false negatives on edge-cases, but that's ok for our purposes
       int i;
       int j;
       bool c = false;
       for (i = 0, j = noVertices() - 1; i < noVertices(); j = i++) {
         if (((vertices_.at(i).y() > point.y()) != (vertices_.at(j).y() > point.y())) &&
             (point.x() < (vertices_.at(j).x() - vertices_.at(i).x()) * (point.y() - vertices_.at(i).y()) / (vertices_.at(j).y() - vertices_.at(i).y()) + vertices_.at(i).x())) {
           {
             c = !c;
           }
         }
       }
       if (c > 0) {
         return true;
       }
   
       // If this statement is reached, the point lies outside the polygon or maybe on its edges
       // Let us check the minium distance as well
       return min_dist == 0 ? false : getMinimumDistance(point) < min_dist;
     }
   
     bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const override;
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Eigen::Vector2d& position) const override { return distance_point_to_polygon_2d(position, vertices_); }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const override { return distance_segment_to_polygon_2d(line_start, line_end, vertices_); }
   
     // implements getMinimumDistance() of the base class
     double getMinimumDistance(const Point2dContainer& polygon) const override { return distance_polygon_to_polygon_2d(polygon, vertices_); }
   
     // implements getMinimumDistanceVec() of the base class
     Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const override;
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const override {
       Point2dContainer pred_vertices;
       predictVertices(t, pred_vertices);
       return distance_point_to_polygon_2d(position, pred_vertices);
     }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const override {
       Point2dContainer pred_vertices;
       predictVertices(t, pred_vertices);
       return distance_segment_to_polygon_2d(line_start, line_end, pred_vertices);
     }
   
     // implements getMinimumSpatioTemporalDistance() of the base class
     double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const override {
       Point2dContainer pred_vertices;
       predictVertices(t, pred_vertices);
       return distance_polygon_to_polygon_2d(polygon, pred_vertices);
     }
   
     virtual void predictVertices(double t, Point2dContainer& pred_vertices) const {
       // Predict obstacle (polygon) at time t
       pred_vertices.resize(vertices_.size());
       Eigen::Vector2d offset = t * centroid_velocity_;
       for (std::size_t i = 0; i < vertices_.size(); i++) {
         pred_vertices[i] = vertices_[i] + offset;
       }
     }
   
     // implements getCentroid() of the base class
     const Eigen::Vector2d& getCentroid() const override {
       assert(finalized_ && "Finalize the polygon after all vertices are added.");
       return centroid_;
     }
   
     // implements getCentroidCplx() of the base class
     std::complex<double> getCentroidCplx() const override {
       assert(finalized_ && "Finalize the polygon after all vertices are added.");
       return std::complex<double>(centroid_.coeffRef(0), centroid_.coeffRef(1));
     }
   
     // implements toPolygonMsg() of the base class
     void toPolygonMsg(geometry_msgs::msg::Polygon& polygon) override;
   
   
     // Access or modify polygon
     const Point2dContainer& vertices() const { return vertices_; }  
     Point2dContainer& vertices() { return vertices_; }              
   
     void pushBackVertex(const Eigen::Ref<const Eigen::Vector2d>& vertex) {
       vertices_.emplace_back(vertex);
       finalized_ = false;
     }
   
     void pushBackVertex(double x, double y) {
       vertices_.emplace_back(x, y);
       finalized_ = false;
     }
   
     void finalizePolygon() {
       fixPolygonClosure();
       calcCentroid();
       finalized_ = true;
     }
   
     void clearVertices() {
       vertices_.clear();
       finalized_ = false;
     }
   
     int noVertices() const { return static_cast<int>(vertices_.size()); }
   
   
    protected:
     void fixPolygonClosure();  
   
     void calcCentroid();  
   
     Point2dContainer vertices_;  
     Eigen::Vector2d centroid_;   
   
     bool finalized_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   }  // namespace hateb_local_planner
   
   #endif /* OBSTACLES_H */
