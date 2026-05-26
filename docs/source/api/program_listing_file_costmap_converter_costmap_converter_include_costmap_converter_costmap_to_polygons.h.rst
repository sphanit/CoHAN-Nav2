
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_polygons.h:

Program Listing for File costmap_to_polygons.h
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_polygons.h>` (``costmap_converter/costmap_converter/include/costmap_converter/costmap_to_polygons.h``)

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
    * Author: Christoph Rösmann, Otniel Rinaldo
    *********************************************************************/
   
   #ifndef COSTMAP_TO_POLYGONS_H_
   #define COSTMAP_TO_POLYGONS_H_
   
   #include <rclcpp/rclcpp.hpp>
   #include <costmap_converter/costmap_converter_interface.h>
   #include <nav_msgs/msg/occupancy_grid.hpp>
   #include <visualization_msgs/msg/marker.hpp>
   #include <geometry_msgs/msg/point.hpp>
   #include <geometry_msgs/msg/polygon.hpp>
   #include <vector>
   #include <algorithm>
   #include <mutex>
   #include <Eigen/Core>
   #include <Eigen/StdVector>
   
   // dynamic reconfigure
   //#include <costmap_converter/CostmapToPolygonsDBSMCCHConfig.h>
   //#include <dynamic_reconfigure/server.h>
   
   
   namespace costmap_converter
   {
     
   class CostmapToPolygonsDBSMCCH : public BaseCostmapToPolygons
   {
     public:
       
       struct KeyPoint
       {
         KeyPoint() : x(0.0), y(0.0) {}
         KeyPoint(double x_, double y_) : x(x_), y(y_) {}
         double x; 
         double y; 
         
         void toPointMsg(geometry_msgs::msg::Point& point) const {point.x=x; point.y=y; point.z=0;}
         void toPointMsg(geometry_msgs::msg::Point32& point) const {point.x=x; point.y=y; point.z=0;}
       };
   
       struct Parameters
       {
         Parameters() : max_distance_(0.4), min_pts_(2), max_pts_(30), min_keypoint_separation_(0.1) {}
         // DBSCAN parameters
         double max_distance_; 
         int min_pts_; 
         int max_pts_; 
         
         // convex hull parameters
         double min_keypoint_separation_; 
       };
       
       CostmapToPolygonsDBSMCCH();
       
       virtual ~CostmapToPolygonsDBSMCCH();
       
       virtual void initialize(rclcpp::Node::SharedPtr nh) override;
       
       virtual void compute();
           
       virtual void setCostmap2D(nav2_costmap_2d::Costmap2D* costmap);
       
       virtual void updateCostmap2D();
       
       
       template< typename Point>
       static void convertPointToPolygon(const Point& point, geometry_msgs::msg::Polygon& polygon)
       {
         polygon.points.resize(1);
         polygon.points.front().x = point.x;
         polygon.points.front().y = point.y;
         polygon.points.front().z = 0;
       }
       
       PolygonContainerConstPtr getPolygons();
   
       
       
     protected:
       
       void dbScan(std::vector< std::vector<KeyPoint> >& clusters);
       
       void regionQuery(int curr_index, std::vector<int>& neighbor_indices);
   
       void addPoint(double x, double y)
       {
         int idx = occupied_cells_.size();
         occupied_cells_.emplace_back(x, y);
         int cx, cy;
         pointToNeighborCells(occupied_cells_.back(), cx, cy);
         int nidx = neighborCellsToIndex(cx, cy);
         if (nidx >= 0)
           neighbor_lookup_[nidx].push_back(idx);
       }
       
       void convexHull(std::vector<KeyPoint>& cluster, geometry_msgs::msg::Polygon& polygon);
       
       void convexHull2(std::vector<KeyPoint>& cluster, geometry_msgs::msg::Polygon& polygon);
   
       void simplifyPolygon(geometry_msgs::msg::Polygon& polygon);
       
       template <typename P1, typename P2, typename P3>
       long double cross(const P1& O, const P2& A, const P3& B)
       { 
           return (long double)(A.x - O.x) * (long double)(B.y - O.y) - (long double)(A.y - O.y) * (long double)(B.x - O.x);
       }
       
         
      void updatePolygonContainer(PolygonContainerPtr polygons);   
             
      
      std::vector<KeyPoint> occupied_cells_; 
      
       std::vector<std::vector<int> > neighbor_lookup_; 
       int neighbor_size_x_; 
       int neighbor_size_y_; 
       double offset_x_;     
       double offset_y_;     
   
       int neighborCellsToIndex(int cx, int cy)
       {
         if (cx < 0 || cx >= neighbor_size_x_ || cy < 0 || cy >= neighbor_size_y_)
           return -1;
         return cy * neighbor_size_x_ + cx;
       }
   
       void pointToNeighborCells(const KeyPoint& kp, int& cx, int& cy)
       {
         cx = int((kp.x - offset_x_) / parameter_.max_distance_);
         cy = int((kp.y - offset_y_) / parameter_.max_distance_);
       }
   
   
       Parameters parameter_;          //< active parameters throughout computation
       Parameters parameter_buffered_; //< the buffered parameters that are offered to dynamic reconfigure
       std::mutex parameter_mutex_;  
      
     private:
          
       //void reconfigureCB(CostmapToPolygonsDBSMCCHConfig& config, uint32_t level);
       
       
       PolygonContainerPtr polygons_; 
       std::mutex mutex_; 
       
       //dynamic_reconfigure::Server<CostmapToPolygonsDBSMCCHConfig>* dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
      
       nav2_costmap_2d::Costmap2D *costmap_; 
      
   }; 
   
     
   } //end namespace teb_local_planner
   
   #endif /* COSTMAP_TO_POLYGONS_H_ */
