
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_dynamic_obstacles_background_subtractor.h:

Program Listing for File background_subtractor.h
================================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_dynamic_obstacles_background_subtractor.h>` (``costmap_converter/costmap_converter/include/costmap_converter/costmap_to_dynamic_obstacles/background_subtractor.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2017
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
    * Notes:
    * The following code makes use of the OpenCV library.
    * OpenCV is licensed under the terms of the 3-clause BSD License.
    *
    * Authors: Franz Albers, Christoph Rösmann
    *********************************************************************/
   
   #ifndef BACKGROUNDSUBTRACTOR_H_
   #define BACKGROUNDSUBTRACTOR_H_
   
   #include <cv_bridge/cv_bridge.h>
   
   class BackgroundSubtractor
   {
   public:
     struct Params
     {
       double alpha_slow; 
       double alpha_fast; 
       double beta;
       double min_sep_between_fast_and_slow_filter;
       double min_occupancy_probability;
       double max_occupancy_neighbors;
       int morph_size;
     };
   
     BackgroundSubtractor(const Params& parameters);
   
     void apply(const cv::Mat& image, cv::Mat& fg_mask, int shift_x = 0, int shift_y = 0);
   
     void visualize(const std::string& name, const cv::Mat& image);
   
     void writeMatToYAML(const std::string& filename, const std::vector<cv::Mat>& mat_vec);
   
     void updateParameters(const Params& parameters);
   
   private:
     void transformToCurrentFrame(int shift_x, int shift_y);
   
     cv::Mat occupancy_grid_fast_;
     cv::Mat occupancy_grid_slow_;
     cv::Mat current_frame_;
   
     int previous_shift_x_;
     int previous_shift_y_;
   
     Params params_;
   };
   
   #endif // BACKGROUNDSUBTRACTOR_H_
