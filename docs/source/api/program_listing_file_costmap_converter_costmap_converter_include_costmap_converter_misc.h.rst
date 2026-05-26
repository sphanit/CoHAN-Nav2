
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_misc.h:

Program Listing for File costmap_converter/costmap_converter/include/costmap_converter/misc.h
=============================================================================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_misc.h>` (``costmap_converter/costmap_converter/include/costmap_converter/misc.h``)

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
   
   #ifndef MISC_H_
   #define MISC_H_
   
   #include <algorithm>
   #include <cmath>
   
   namespace costmap_converter
   {
   
   template <typename Point, typename LinePoint>
   inline double computeDistanceToLine(const Point& point, const LinePoint& line_pt1, const LinePoint& line_pt2)
   {
       double dx = line_pt2.x - line_pt1.x;
       double dy = line_pt2.y - line_pt1.y;
       
       double length = std::sqrt(dx*dx + dy*dy);
       
       if (length>0)
         return std::abs(dy * point.x - dx * point.y + line_pt2.x * line_pt1.y - line_pt2.y * line_pt1.x) / length;
     
       return std::sqrt(std::pow(point.x - line_pt1.x, 2) + std::pow(point.y - line_pt1.y, 2));  
   }
   
   
   template <typename Point, typename LinePoint>
   inline double computeSquaredDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end, bool* is_inbetween=NULL)
   {
       double dx = line_end.x - line_start.x;
       double dy = line_end.y - line_start.y;
       
       double length_sqr = dx*dx + dy*dy;
   
       double u = 0;
   
       if (length_sqr > 0)
         u = ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / length_sqr;
   
       if (is_inbetween)
         *is_inbetween = (u>=0 && u<=1);
     
       if (u <= 0)
         return std::pow(point.x-line_start.x,2) + std::pow(point.y-line_start.y,2);
       
       if (u >= 1)
         return std::pow(point.x-line_end.x,2) + std::pow(point.y-line_end.y,2);
       
       return std::pow(point.x - (line_start.x+u*dx) ,2) + std::pow(point.y - (line_start.y+u*dy),2);
   }
     
   template <typename Point, typename LinePoint>
   inline double computeDistanceToLineSegment(const Point& point, const LinePoint& line_start, const LinePoint& line_end, bool* is_inbetween=NULL)
   {
     return std::sqrt(computeSquaredDistanceToLineSegment(point, line_start, line_end, is_inbetween));
   }
     
   
   template <typename Point1, typename Point2>  
   inline double norm2d(const Point1& pt1, const Point2& pt2)
   {
     return std::sqrt( std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2)  );
   }
   
   template <typename Point1, typename Point2>  
   inline bool isApprox2d(const Point1& pt1, const Point2& pt2, double threshold)
   {
     return ( std::abs(pt2.x-pt1.x)<threshold && std::abs(pt2.y-pt1.y)<threshold );
   }
   
   
     
   } //end namespace teb_local_planner
   
   #endif /* MISC_H_ */
