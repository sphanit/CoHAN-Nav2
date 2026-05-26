
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_dynamic_obstacles_multitarget_tracker_defines.h:

Program Listing for File defines.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_dynamic_obstacles_multitarget_tracker_defines.h>` (``costmap_converter/costmap_converter/include/costmap_converter/costmap_to_dynamic_obstacles/multitarget_tracker/defines.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   // Based on https://github.com/Smorodov/Multitarget-tracker/tree/master/Tracker, GPLv3
   // Refer to README.md in this directory.
   
   #pragma once
   #include <opencv2/opencv.hpp>
   
   typedef float track_t;
   typedef cv::Point3_<track_t> Point_t;
   #define Mat_t CV_32FC
