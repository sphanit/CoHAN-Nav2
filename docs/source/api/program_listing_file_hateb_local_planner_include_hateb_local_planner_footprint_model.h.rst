
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_footprint_model.h:

Program Listing for File footprint_model.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_footprint_model.h>` (``hateb_local_planner/include/hateb_local_planner/footprint_model.h``)

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
   
   #ifndef ROBOT_FOOTPRINT_MODEL_H
   #define ROBOT_FOOTPRINT_MODEL_H
   
   #include <hateb_local_planner/obstacles.h>
   #include <hateb_local_planner/pose_se2.h>
   
   #include <algorithm>
   #include <utility>
   #include <visualization_msgs/msg/marker.hpp>
   
   namespace hateb_local_planner {
   
   class BaseFootprintModel {
    public:
     BaseFootprintModel() = default;
   
     virtual ~BaseFootprintModel() = default;
   
     virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const = 0;
   
     virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const = 0;
   
     virtual void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const {}
   
     virtual double getInscribedRadius() = 0;
   
     virtual double getCircumscribedRadius() const = 0;
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   using FootprintModelPtr = std::shared_ptr<BaseFootprintModel>;
   using FootprintModelConstPtr = std::shared_ptr<const BaseFootprintModel>;
   
   class PointFootprint : public BaseFootprintModel {
    public:
     PointFootprint() = default;
   
     explicit PointFootprint(const double min_obstacle_dist) : min_obstacle_dist_(min_obstacle_dist) {}
   
     ~PointFootprint() override = default;
   
     double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override { return obstacle->getMinimumDistance(current_pose.position()); }
   
     double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
       return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t);
     }
   
     double getInscribedRadius() override { return 0.0; }
   
     double getCircumscribedRadius() const override { return 0.0; }
   
     void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
       // point footprint
       markers.emplace_back();
       visualization_msgs::msg::Marker& marker = markers.back();
       marker.type = visualization_msgs::msg::Marker::POINTS;
       current_pose.toPoseMsg(marker.pose);  // all points are transformed into the robot/human frame!
       marker.points.emplace_back();
       marker.scale.x = 0.025;
       marker.color = color;
   
       if (min_obstacle_dist_ <= 0) {
         return;
       }
   
       // footprint with min_obstacle_dist
       markers.emplace_back();
       visualization_msgs::msg::Marker& marker2 = markers.back();
       marker2.type = visualization_msgs::msg::Marker::LINE_STRIP;
       marker2.scale.x = 0.025;
       marker2.color = color;
       current_pose.toPoseMsg(marker2.pose);  // all points are transformed into the robot/human frame!
   
       const double n = 9;
       const double r = min_obstacle_dist_;
       for (double theta = 0; theta <= 2 * M_PI; theta += M_PI / n) {
         geometry_msgs::msg::Point pt;
         pt.x = r * cos(theta);
         pt.y = r * sin(theta);
         marker2.points.push_back(pt);
       }
     }
   
    private:
     const double min_obstacle_dist_ = 0.0;
   };
   
   class CircularFootprint : public BaseFootprintModel {
    public:
     CircularFootprint() = default;
     explicit CircularFootprint(double radius) : radius_(radius) {}
   
     ~CircularFootprint() override = default;
   
     void setRadius(double radius) { radius_ = radius; }
   
     double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override { return obstacle->getMinimumDistance(current_pose.position()) - radius_; }
   
     double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
       return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t) - radius_;
     }
   
     void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
       markers.resize(1);
       visualization_msgs::msg::Marker& marker = markers.back();
       marker.type = visualization_msgs::msg::Marker::CYLINDER;
       current_pose.toPoseMsg(marker.pose);
       marker.scale.x = marker.scale.y = 2 * radius_;  // scale = diameter
       marker.scale.z = 0.05;
       marker.color = color;
     }
   
     double getInscribedRadius() override { return radius_; }
   
     double getCircumscribedRadius() const override { return radius_; }
   
    private:
     double radius_;
   };
   using CircularFootprintPtr = std::shared_ptr<CircularFootprint>;
   
   class TwoCirclesFootprint : public BaseFootprintModel {
    public:
     TwoCirclesFootprint(double front_offset, double front_radius, double rear_offset, double rear_radius)
         : front_offset_(front_offset), front_radius_(front_radius), rear_offset_(rear_offset), rear_radius_(rear_radius) {}
   
     ~TwoCirclesFootprint() override = default;
   
     void setParameters(double front_offset, double front_radius, double rear_offset, double rear_radius) {
       front_offset_ = front_offset;
       front_radius_ = front_radius;
       rear_offset_ = rear_offset;
       rear_radius_ = rear_radius;
     }
   
     double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override {
       Eigen::Vector2d dir = current_pose.orientationUnitVec();
       double dist_front = obstacle->getMinimumDistance(current_pose.position() + front_offset_ * dir) - front_radius_;
       double dist_rear = obstacle->getMinimumDistance(current_pose.position() - rear_offset_ * dir) - rear_radius_;
       return std::min(dist_front, dist_rear);
     }
   
     double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
       Eigen::Vector2d dir = current_pose.orientationUnitVec();
       double dist_front = obstacle->getMinimumSpatioTemporalDistance(current_pose.position() + front_offset_ * dir, t) - front_radius_;
       double dist_rear = obstacle->getMinimumSpatioTemporalDistance(current_pose.position() - rear_offset_ * dir, t) - rear_radius_;
       return std::min(dist_front, dist_rear);
     }
   
     void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
       Eigen::Vector2d dir = current_pose.orientationUnitVec();
       if (front_radius_ > 0) {
         markers.emplace_back();
         visualization_msgs::msg::Marker& marker1 = markers.front();
         marker1.type = visualization_msgs::msg::Marker::CYLINDER;
         current_pose.toPoseMsg(marker1.pose);
         marker1.pose.position.x += front_offset_ * dir.x();
         marker1.pose.position.y += front_offset_ * dir.y();
         marker1.scale.x = marker1.scale.y = 2 * front_radius_;  // scale = diameter
                                                                 //       marker1.scale.z = 0.05;
         marker1.color = color;
       }
       if (rear_radius_ > 0) {
         markers.emplace_back();
         visualization_msgs::msg::Marker& marker2 = markers.back();
         marker2.type = visualization_msgs::msg::Marker::CYLINDER;
         current_pose.toPoseMsg(marker2.pose);
         marker2.pose.position.x -= rear_offset_ * dir.x();
         marker2.pose.position.y -= rear_offset_ * dir.y();
         marker2.scale.x = marker2.scale.y = 2 * rear_radius_;  // scale = diameter
                                                                //       marker2.scale.z = 0.05;
         marker2.color = color;
       }
     }
   
     double getInscribedRadius() override {
       double min_longitudinal = std::min(rear_offset_ + rear_radius_, front_offset_ + front_radius_);
       double min_lateral = std::min(rear_radius_, front_radius_);
       return std::min(min_longitudinal, min_lateral);
     }
   
     double getCircumscribedRadius() const override { return std::max(front_offset_ + front_radius_, rear_offset_ + rear_radius_); }
   
    private:
     double front_offset_;
     double front_radius_;
     double rear_offset_;
     double rear_radius_;
   };
   
   class LineFootprint : public BaseFootprintModel {
    public:
     LineFootprint(const geometry_msgs::msg::Point& line_start, const geometry_msgs::msg::Point& line_end) { setLine(line_start, line_end); }
   
     LineFootprint(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, const double min_obstacle_dist) : min_obstacle_dist_(min_obstacle_dist) { setLine(line_start, line_end); }
   
     ~LineFootprint() override = default;
   
     void setLine(const geometry_msgs::msg::Point& line_start, const geometry_msgs::msg::Point& line_end) {
       line_start_.x() = line_start.x;
       line_start_.y() = line_start.y;
       line_end_.x() = line_end.x;
       line_end_.y() = line_end.y;
     }
   
     void setLine(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) {
       line_start_ = line_start;
       line_end_ = line_end;
     }
   
     double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override {
       Eigen::Vector2d line_start_world;
       Eigen::Vector2d line_end_world;
       transformToWorld(current_pose, line_start_world, line_end_world);
       return obstacle->getMinimumDistance(line_start_world, line_end_world);
     }
   
     double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
       Eigen::Vector2d line_start_world;
       Eigen::Vector2d line_end_world;
       transformToWorld(current_pose, line_start_world, line_end_world);
       return obstacle->getMinimumSpatioTemporalDistance(line_start_world, line_end_world, t);
     }
   
     void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
       markers.emplace_back();
       visualization_msgs::msg::Marker& marker = markers.front();
       marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
       current_pose.toPoseMsg(marker.pose);  // all points are transformed into the robot/human frame!
   
       // line
       geometry_msgs::msg::Point line_start_world;
       line_start_world.x = line_start_.x();
       line_start_world.y = line_start_.y();
       line_start_world.z = 0;
       marker.points.push_back(line_start_world);
   
       geometry_msgs::msg::Point line_end_world;
       line_end_world.x = line_end_.x();
       line_end_world.y = line_end_.y();
       line_end_world.z = 0;
       marker.points.push_back(line_end_world);
   
       marker.scale.x = 0.05;
       marker.color = color;
     }
   
     double getInscribedRadius() override {
       return 0.0;  // lateral distance = 0.0
     }
   
     double getCircumscribedRadius() const override { return std::max(std::hypot(line_start_.x(), line_start_.y()), std::hypot(line_end_.x(), line_end_.y())); }
   
    private:
     void transformToWorld(const PoseSE2& current_pose, Eigen::Vector2d& line_start_world, Eigen::Vector2d& line_end_world) const {
       double cos_th = std::cos(current_pose.theta());
       double sin_th = std::sin(current_pose.theta());
       line_start_world.x() = current_pose.x() + cos_th * line_start_.x() - sin_th * line_start_.y();
       line_start_world.y() = current_pose.y() + sin_th * line_start_.x() + cos_th * line_start_.y();
       line_end_world.x() = current_pose.x() + cos_th * line_end_.x() - sin_th * line_end_.y();
       line_end_world.y() = current_pose.y() + sin_th * line_end_.x() + cos_th * line_end_.y();
     }
   
     Eigen::Vector2d line_start_;
     Eigen::Vector2d line_end_;
     const double min_obstacle_dist_ = 0.0;
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class PolygonFootprint : public BaseFootprintModel {
    public:
     explicit PolygonFootprint(Point2dContainer vertices) : vertices_(std::move(vertices)) {}
   
     ~PolygonFootprint() override = default;
   
     void setVertices(const Point2dContainer& vertices) { vertices_ = vertices; }
   
     double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const override {
       Point2dContainer polygon_world(vertices_.size());
       transformToWorld(current_pose, polygon_world);
       return obstacle->getMinimumDistance(polygon_world);
     }
   
     double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const override {
       Point2dContainer polygon_world(vertices_.size());
       transformToWorld(current_pose, polygon_world);
       return obstacle->getMinimumSpatioTemporalDistance(polygon_world, t);
     }
   
     void visualizeModel(const PoseSE2& current_pose, std::vector<visualization_msgs::msg::Marker>& markers, const std_msgs::msg::ColorRGBA& color) const override {
       if (vertices_.empty()) return;
   
       markers.emplace_back();
       visualization_msgs::msg::Marker& marker = markers.front();
       marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
       current_pose.toPoseMsg(marker.pose);  // all points are transformed into the robot/human frame!
   
       for (const auto& vertice : vertices_) {
         geometry_msgs::msg::Point point;
         point.x = vertice.x();
         point.y = vertice.y();
         point.z = 0;
         marker.points.push_back(point);
       }
       // add first point again in order to close the polygon
       geometry_msgs::msg::Point point;
       point.x = vertices_.front().x();
       point.y = vertices_.front().y();
       point.z = 0;
       marker.points.push_back(point);
   
       marker.scale.x = 0.025;
       marker.color = color;
     }
   
     double getInscribedRadius() override {
       double min_dist = std::numeric_limits<double>::max();
       Eigen::Vector2d center(0.0, 0.0);
   
       if (vertices_.size() <= 2) {
         return 0.0;
       }
   
       for (int i = 0; i < static_cast<int>(vertices_.size()) - 1; ++i) {
         // compute distance from the robot/human center point to the first vertex
         double vertex_dist = vertices_[i].norm();
         double edge_dist = distance_point_to_segment_2d(center, vertices_[i], vertices_[i + 1]);
         min_dist = std::min({min_dist, vertex_dist, edge_dist});
       }
   
       // we also need to check the last vertex and the first vertex
       double vertex_dist = vertices_.back().norm();
       double edge_dist = distance_point_to_segment_2d(center, vertices_.back(), vertices_.front());
       return std::min({min_dist, vertex_dist, edge_dist});
     }
   
     double getCircumscribedRadius() const override {
       double radius = 0.0;
       for (const auto& vertex : vertices_) {
         double dist = std::hypot(vertex.x(), vertex.y());
         radius = std::max(radius, dist);
       }
       return radius;
     }
   
    private:
     void transformToWorld(const PoseSE2& current_pose, Point2dContainer& polygon_world) const {
       double cos_th = std::cos(current_pose.theta());
       double sin_th = std::sin(current_pose.theta());
       for (std::size_t i = 0; i < vertices_.size(); ++i) {
         polygon_world[i].x() = current_pose.x() + cos_th * vertices_[i].x() - sin_th * vertices_[i].y();
         polygon_world[i].y() = current_pose.y() + sin_th * vertices_[i].x() + cos_th * vertices_[i].y();
       }
     }
   
     Point2dContainer vertices_;
   };
   
   }  // namespace hateb_local_planner
   
   #endif /* ROBOT_FOOTPRINT_MODEL_H */
