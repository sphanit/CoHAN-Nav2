
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_lines_convex_hull.h:

Program Listing for File costmap_to_lines_convex_hull.h
=======================================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_lines_convex_hull.h>` (``costmap_converter/costmap_converter/include/costmap_converter/costmap_to_lines_convex_hull.h``)

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
   
   #ifndef COSTMAP_TO_LINES_CONVEX_HULL_H_
   #define COSTMAP_TO_LINES_CONVEX_HULL_H_
   
   #include <costmap_converter/costmap_converter_interface.h>
   #include <costmap_converter/costmap_to_polygons.h>
   
   // dynamic reconfigure
   //#include <costmap_converter/CostmapToLinesDBSMCCHConfig.h>
   
   namespace costmap_converter
   {
     
     class CostmapToLinesDBSMCCH : public CostmapToPolygonsDBSMCCH
     {
     public:
      
       CostmapToLinesDBSMCCH();
          
       virtual ~CostmapToLinesDBSMCCH();
       
       virtual void initialize(rclcpp::Node::SharedPtr nh);
       
       virtual void compute();   
       
       
     protected:
       
       void extractPointsAndLines(std::vector<KeyPoint>& cluster, const geometry_msgs::msg::Polygon& polygon, std::back_insert_iterator< std::vector<geometry_msgs::msg::Polygon> > lines);
   
       
       
     protected:
         
       double support_pts_max_dist_inbetween_;
       double support_pts_max_dist_;
       int min_support_pts_;
      
     private:
       
   //    void reconfigureCB(CostmapToLinesDBSMCCHConfig& config, uint32_t level);
       
   //    dynamic_reconfigure::Server<CostmapToLinesDBSMCCHConfig>* dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
    
     };  
     
     
     
     
   
   
     
     
   } //end namespace teb_local_planner
   
   #endif /* COSTMAP_TO_LINES_CONVEX_HULL_H_ */
