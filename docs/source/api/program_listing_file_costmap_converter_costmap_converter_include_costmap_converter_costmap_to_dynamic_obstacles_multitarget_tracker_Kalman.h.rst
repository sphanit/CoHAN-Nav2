
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_dynamic_obstacles_multitarget_tracker_Kalman.h:

Program Listing for File Kalman.h
=================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_dynamic_obstacles_multitarget_tracker_Kalman.h>` (``costmap_converter/costmap_converter/include/costmap_converter/costmap_to_dynamic_obstacles/multitarget_tracker/Kalman.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   // Based on https://github.com/Smorodov/Multitarget-tracker/tree/master/Tracker, GPLv3
   // Refer to README.md in this directory.
   
   #pragma once
   #include "defines.h"
   #include <opencv2/opencv.hpp>
   
   // http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
   class TKalmanFilter
   {
   public:
     TKalmanFilter(Point_t p, track_t deltatime = 0.2);
     ~TKalmanFilter();
     void Prediction();
     Point_t Update(Point_t p, bool DataCorrect);
     cv::KalmanFilter* kalman;
     track_t dt;
     Point_t LastPosition; // contour in [px]
     Point_t LastVelocity; // velocity in [px/s]
   };
