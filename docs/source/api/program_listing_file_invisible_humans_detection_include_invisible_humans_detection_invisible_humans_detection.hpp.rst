
.. _program_listing_file_invisible_humans_detection_include_invisible_humans_detection_invisible_humans_detection.hpp:

Program Listing for File invisible_humans_detection.hpp
=======================================================

|exhale_lsh| :ref:`Return to documentation for file <file_invisible_humans_detection_include_invisible_humans_detection_invisible_humans_detection.hpp>` (``invisible_humans_detection/include/invisible_humans_detection/invisible_humans_detection.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*******************************************************************************
    * Software License Agreement (MIT License)
    *
    * Copyright (c) 2022-2025 LAAS-CNRS
    *
    * Permission is hereby granted, free of charge, to any person obtaining a copy
    * of this software and associated documentation files (the "Software"), to deal
    * in the Software without restriction, including without limitation the rights
    * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    * copies of the Software, and to permit persons to whom the Software is
    * furnished to do so, subject to the following conditions:
    *
    * The above copyright notice and this permission notice shall be included in
    * all copies or substantial portions of the Software.
    *
    * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    * THE SOFTWARE.
    *
    * Author: Phani Teja Singamaneni
    *********************************************************************************/
   
   #ifndef INVISIBLE_HUMANS_DETECTION_HPP
   #define INVISIBLE_HUMANS_DETECTION_HPP
   
   #include <tf2/LinearMath/Quaternion.h>
   #include <tf2/LinearMath/Transform.h>
   #include <tf2/LinearMath/Vector3.h>
   #include <tf2/utils.h>
   #include <tf2_ros/buffer.h>
   #include <tf2_ros/transform_listener.h>
   
   #include <Eigen/Core>
   #include <cassert>
   #include <cmath>
   #include <cohan_msgs/msg/passage_type.hpp>
   #include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
   #include <costmap_converter_msgs/msg/obstacle_msg.hpp>
   #include <geometry_msgs/msg/pose_array.hpp>
   #include <geometry_msgs/msg/pose_stamped.hpp>
   #include <geometry_msgs/msg/transform_stamped.hpp>
   #include <invisible_humans_detection/invisible_humans_config.hpp>
   #include <map>
   #include <memory>
   #include <nav_msgs/msg/occupancy_grid.hpp>
   #include <rclcpp/rclcpp.hpp>
   #include <sensor_msgs/msg/laser_scan.hpp>
   #include <std_msgs/msg/int8.hpp>
   #include <string>
   #include <tf2_eigen/tf2_eigen.hpp>
   #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
   #include <vector>
   #include <visualization_msgs/msg/marker.hpp>
   #include <visualization_msgs/msg/marker_array.hpp>
   
   namespace invisible_humans_detection {
   using Coordinates = std::vector<std::pair<double, double>>;
   using Point = std::pair<double, double>;
   
   // Pattern 1 of ROS2 Nodes --> Inherit from rclcpp::Node (Good for Standalone nodes)
   
   class InvHumansDetection : public rclcpp::Node {
    public:
     InvHumansDetection() : Node("inv_humans_detection") {}
   
     void initialize();
   
     ~InvHumansDetection() = default;
   
    private:
     void mapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);
   
     bool locateInvHumans(Coordinates c1, Coordinates c2, std::vector<char> direction, geometry_msgs::msg::TransformStamped& footprint_transform);
   
     void detectOccludedCorners();
   
     void publishInvisibleHumans(const geometry_msgs::msg::PoseArray& corners, const geometry_msgs::msg::PoseArray& poses, std::vector<std::vector<double>>& inv_humans);
   
     void detectPassages(geometry_msgs::msg::PoseArray detections);
   
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
   
     // ROS2 helpers and configuration
     std::shared_ptr<InvisibleHumansConfig> cfg_;  
   
     rclcpp::TimerBase::SharedPtr timer_;                                                           
     geometry_msgs::msg::PoseStamped robot_pose_;                                                   
     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                                   
     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                                      
     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;                        
     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;                           
     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_invis_human_viz_;       
     rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr pub_invis_human_;  
     rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_invis_human_corners_;          
     rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_invis_humans_pos_;             
     rclcpp::Publisher<cohan_msgs::msg::PassageType>::SharedPtr passage_detect_pub_;                
     nav_msgs::msg::OccupancyGrid map_;                                                             
     std::vector<float> ranges_;                                                                    
     std::vector<double> corner_ranges_;                                                            
     int size_x_;                                                                                   
     int size_y_;                                                                                   
     double origin_x_;                                                                              
     double origin_y_;                                                                              
     double resolution_;                                                                            
     sensor_msgs::msg::LaserScan scan_msg_;                                                         
     Eigen::Vector2d robot_vec_;                                                                    
   };
   
   }  // namespace invisible_humans_detection
   
   #endif  // INVISIBLE_HUMANS_DETECTION_HPP
