
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_lines_ransac.h:

Program Listing for File costmap_to_lines_ransac.h
==================================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_lines_ransac.h>` (``costmap_converter/costmap_converter/include/costmap_converter/costmap_to_lines_ransac.h``)

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
   
   #ifndef COSTMAP_TO_LINES_RANSAC_H_
   #define COSTMAP_TO_LINES_RANSAC_H_
   
   #include <costmap_converter/costmap_converter_interface.h>
   #include <costmap_converter/costmap_to_polygons.h>
   #include <costmap_converter/misc.h>
   
   #include <random>
   
   //#include <costmap_converter/CostmapToLinesDBSRANSACConfig.h>
   
   namespace costmap_converter
   {
     
     class CostmapToLinesDBSRANSAC : public CostmapToPolygonsDBSMCCH
     {
     public:
      
       CostmapToLinesDBSRANSAC();
          
       virtual ~CostmapToLinesDBSRANSAC();
       
       virtual void initialize(rclcpp::Node::SharedPtr nh);
       
       virtual void compute();   
           
        
       template <typename Point, typename LinePoint>
       static bool isInlier(const Point& point, const LinePoint& line_start, const LinePoint& line_end, double min_distance);
       
     protected:
       
       double ransac_inlier_distance_; 
       int ransac_min_inliers_; 
       int ransac_no_iterations_; 
       int ransac_remainig_outliers_; 
       bool ransac_convert_outlier_pts_; 
       bool ransac_filter_remaining_outlier_pts_; 
      
      
      std::mt19937 rnd_generator_; 
       
       
       bool lineRansac(const std::vector<KeyPoint>& data, double inlier_distance, int no_iterations, int min_inliers,
                       std::pair<KeyPoint, KeyPoint>& best_model, std::vector<KeyPoint>* inliers = NULL,
                        std::vector<KeyPoint>* outliers = NULL);
       
       bool linearRegression(const std::vector<KeyPoint>& data, double& slope, double& intercept,
                             double* mean_x_out = NULL, double* mean_y_out = NULL);
       
       
       
   //     void adjustLineLength(const std::vector<KeyPoint>& data, const KeyPoint& linept1, const KeyPoint& linept2, 
   //                           KeyPoint& line_start, KeyPoint& line_end);
       
       private:
       
   //    void reconfigureCB(CostmapToLinesDBSRANSACConfig& config, uint32_t level);
       
   //    dynamic_reconfigure::Server<CostmapToLinesDBSRANSACConfig>* dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
     };  
     
     
   
   template <typename Point, typename LinePoint>
   bool CostmapToLinesDBSRANSAC::isInlier(const Point& point, const LinePoint& line_start, const LinePoint& line_end, double min_distance)
   {
     bool is_inbetween = false;
     double distance = computeDistanceToLineSegment(point, line_start, line_end, &is_inbetween);
     if (!is_inbetween)
       return false;
     if (distance <= min_distance)
       return true;
     return false;
   }
        
     
   } //end namespace teb_local_planner
   
   #endif /* COSTMAP_TO_LINES_RANSAC_H_ */
