
.. _program_listing_file_invisible_humans_detection_include_invisible_humans_detection_map_scanner.h:

Program Listing for File map_scanner.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_invisible_humans_detection_include_invisible_humans_detection_map_scanner.h>` (``invisible_humans_detection/include/invisible_humans_detection/map_scanner.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    * Copyright (c) 2022 LAAS/CNRS
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
   #ifndef MAP_SCANNER_H
   #define MAP_SCANNER_H
   #include <cohan_msgs/PassageType.h>
   #include <costmap_converter/ObstacleArrayMsg.h>
   #include <costmap_converter/ObstacleMsg.h>
   #include <geometry_msgs/PoseArray.h>
   #include <nav_msgs/OccupancyGrid.h>
   #include <ros/ros.h>
   #include <sensor_msgs/LaserScan.h>
   #include <std_msgs/Int8.h>
   #include <tf/tf.h>
   #include <tf2/utils.h>
   #include <tf2_eigen/tf2_eigen.h>
   #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
   #include <tf2_ros/transform_listener.h>
   #include <visualization_msgs/Marker.h>
   #include <visualization_msgs/MarkerArray.h>
   
   #include <cassert>
   #include <cmath>
   
   namespace invisible_humans_detection {
   using Coordinates = std::vector<std::pair<double, double>>;
   using Point = std::pair<double, double>;
   
   class MapScanner {
    public:
     MapScanner();
   
     ~MapScanner();
   
     void initialize();
   
    private:
     void loadRosParamFromNodeHandle(const ros::NodeHandle& private_nh);
   
     void mapCB(const nav_msgs::OccupancyGrid& grid);
   
     bool locateInvHumans(Coordinates c1, Coordinates c2, std::vector<char> direction, geometry_msgs::TransformStamped& footprint_transform);
   
     void detectOccludedCorners(const ros::TimerEvent& event);
   
     void publishInvisibleHumans(const geometry_msgs::PoseArray& corners, const geometry_msgs::PoseArray& poses, std::vector<std::vector<double>>& inv_humans);
   
     void detectPassages(geometry_msgs::PoseArray detections);
   
     bool worldToMap(double wx, double wy, int& mx, int& my) const {
       if (wx < origin_x_ || wy < origin_y_) return false;
   
       mx = static_cast<int>((wx - origin_x_) / resolution_);
       my = static_cast<int>((wy - origin_y_) / resolution_);
   
       return mx < size_x_ && my < size_y_;
     }
   
     unsigned int getIndex(unsigned int mx, unsigned int my) const { return (my * size_x_) + mx; }
   
     static Point getLeftPoint(Point p1, Point p2, Point p3, double dist = 1) {
       double x = p2.first - p1.first;
       double y = p2.second - p1.second;
       double point_dist = std::hypot(x, y);
   
       Point p;
       p.first = p3.first - (dist * y / point_dist);
       p.second = p3.second + (dist * x / point_dist);
       return p;
     }
   
     static Point getRightPoint(Point p1, Point p2, Point p3, double dist = 1) {
       double x = p2.first - p1.first;
       double y = p2.second - p1.second;
       double point_dist = std::hypot(x, y);
   
       Point p;
       p.first = p3.first + (dist * y / point_dist);
       p.second = p3.second - (dist * x / point_dist);
       return p;
     }
   
     static std::vector<Point> getTwoPoints(Point p1, Point p2, double radius) {
       std::vector<Point> points;
       auto l_p = getLeftPoint(p1, p2, p2, radius);
       points.push_back(l_p);
       auto r_p = getRightPoint(p1, p2, p2, radius);
       points.push_back(r_p);
       return points;
     }
   
     ros::Timer get_robot_pose_;               
     geometry_msgs::PoseStamped robot_pose_;   
     tf2_ros::Buffer tf_;                      
     ros::Subscriber map_sub_;                 
     ros::Publisher scan_pub_;                 
     ros::Publisher pub_invis_human_viz_;      
     ros::Publisher pub_invis_human_;          
     ros::Publisher pub_invis_human_corners_;  
     ros::Publisher pub_invis_humans_pos_;     
     ros::Publisher passage_detect_pub_;       
     nav_msgs::OccupancyGrid map_;             
     std::vector<float> ranges_;               
     std::vector<double> corner_ranges_;       
     int samples_;                             
     int scan_resolution_;                     
     int size_x_;                              
     int size_y_;                              
     double origin_x_;                         
     double origin_y_;                         
     double resolution_;                       
     double angle_min_;                        
     double angle_max_;                        
     double range_min_;                        
     double range_max_;                        
     sensor_msgs::LaserScan scan_msg_;         
     bool publish_scan_;                       
     double human_radius_;                     
     std::string ns_;                          
     Eigen::Vector2d robot_vec_;               
   };
   
   }  // namespace invisible_humans_detection
   
   #endif  // MAP_SCANNER_H
