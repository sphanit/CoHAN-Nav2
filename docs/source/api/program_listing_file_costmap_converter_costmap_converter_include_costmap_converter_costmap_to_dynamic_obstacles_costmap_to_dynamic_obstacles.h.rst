
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_dynamic_obstacles_costmap_to_dynamic_obstacles.h:

Program Listing for File costmap_to_dynamic_obstacles.h
=======================================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_costmap_to_dynamic_obstacles_costmap_to_dynamic_obstacles.h>` (``costmap_converter/costmap_converter/include/costmap_converter/costmap_to_dynamic_obstacles/costmap_to_dynamic_obstacles.h``)

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
   
   #ifndef COSTMAP_TO_DYNAMIC_OBSTACLES_H_
   #define COSTMAP_TO_DYNAMIC_OBSTACLES_H_
   
   // ROS
   #include <costmap_converter/costmap_converter_interface.h>
   #include <nav_msgs/msg/odometry.hpp>
   #include <pluginlib/class_loader.hpp>
   #include <rclcpp/rclcpp.hpp>
   
   // OpenCV
   #include <cv_bridge/cv_bridge.h>
   #include <opencv2/features2d/features2d.hpp>
   #include <opencv2/video/tracking.hpp>
   
   // dynamic reconfigure
   //#include <costmap_converter/CostmapToDynamicObstaclesConfig.h>
   //#include <dynamic_reconfigure/server.h>
   
   // Own includes
   #include <costmap_converter/costmap_to_dynamic_obstacles/multitarget_tracker/Ctracker.h>
   #include <costmap_converter/costmap_to_dynamic_obstacles/background_subtractor.h>
   #include <costmap_converter/costmap_to_dynamic_obstacles/blob_detector.h>
   
   // STL
   #include <memory>
   
   namespace costmap_converter {
   
   class CostmapToDynamicObstacles : public BaseCostmapToDynamicObstacles
   {
   public:
     CostmapToDynamicObstacles();
   
     virtual ~CostmapToDynamicObstacles();
   
     virtual void initialize(rclcpp::Node::SharedPtr nh);
   
     virtual void compute();
   
     virtual void setCostmap2D(nav2_costmap_2d::Costmap2D* costmap);
   
     virtual void updateCostmap2D();
   
     ObstacleArrayConstPtr getObstacles();
   
     virtual void setOdomTopic(const std::string& odom_topic)
     {
       odom_topic_ = odom_topic;
     }
   
     void visualize(const std::string& name, const cv::Mat& image);
   
   protected:
     Point_t getEstimatedVelocityOfObject(unsigned int idx);
   
     void getContour(unsigned int idx, std::vector<Point_t>& contour);
   
     void updateObstacleContainer(ObstacleArrayPtr obstacles);
   
   private:
     std::mutex mutex_;
     nav2_costmap_2d::Costmap2D* costmap_;
     cv::Mat costmap_mat_;
     ObstacleArrayPtr obstacles_;
     cv::Mat fg_mask_;
     std::unique_ptr<BackgroundSubtractor> bg_sub_;
     cv::Ptr<BlobDetector> blob_det_;
     std::vector<cv::KeyPoint> keypoints_;
     std::unique_ptr<CTracker> tracker_;
     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
     Point_t ego_vel_;
   
     std::string odom_topic_ = "/odom";
     bool publish_static_obstacles_ = true;
   
   //  dynamic_reconfigure::Server<CostmapToDynamicObstaclesConfig>*
   //      dynamic_recfg_; //!< Dynamic reconfigure server to allow config
   //                       //! modifications at runtime
   
     void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
   
   //  void reconfigureCB(CostmapToDynamicObstaclesConfig &config, uint32_t level);
   };
   
   } // end namespace costmap_converter
   
   #endif /* COSTMAP_TO_DYNAMIC_OBSTACLES_H_ */
