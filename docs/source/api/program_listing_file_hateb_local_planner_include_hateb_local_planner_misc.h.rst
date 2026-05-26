
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_misc.h:

Program Listing for File hateb_local_planner/include/hateb_local_planner/misc.h
===============================================================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_misc.h>` (``hateb_local_planner/include/hateb_local_planner/misc.h``)

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
   
   #ifndef MISC_H
   #define MISC_H
   
   #include <Eigen/Core>
   #include <boost/type_traits.hpp>
   #include <boost/utility.hpp>
   #include <vector>
   
   namespace hateb_local_planner {
   
   #define SMALL_NUM 0.00000001
   
   enum class RotType : std::uint8_t { left, none, right };
   
   inline bool is_close(double a, double b, double epsilon = 1e-4) { return std::fabs(a - b) < epsilon; }
   
   inline double average_angles(const std::vector<double>& angles) {
     double x = 0;
     double y = 0;
     for (double angle : angles) {
       x += cos(angle);
       y += sin(angle);
     }
     if (x == 0 && y == 0) {
       return 0;
     }
     return std::atan2(y, x);
   }
   
   inline bool smaller_than_abs(double i, double j) { return std::fabs(i) < std::fabs(j); }
   
   inline double fast_sigmoid(double x) { return x / (1 + fabs(x)); }
   
   inline double center_radians(double angle) {
     while (angle > M_PI) {
       angle -= 2 * M_PI;
     }
     while (angle < -M_PI) {
       angle += 2 * M_PI;
     }
     return angle;
   }
   
   template <typename P1, typename P2>
   inline double distance_points2d(const P1& point1, const P2& point2) {
     return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
   }
   
   template <typename V1, typename V2>
   inline double cross2d(const V1& v1, const V2& v2) {
     return (v1.x() * v2.y()) - (v2.x() * v1.y());
   }
   
   template <typename T>
   inline const T& get_const_reference(const T* ptr) {
     return *ptr;
   }
   
   template <typename T>
   inline const T& get_const_reference(const T& val, typename boost::disable_if<boost::is_pointer<T> >::type* dummy = 0) {
     return val;
   }
   
   }  // namespace hateb_local_planner
   
   #endif /* MISC_H */
