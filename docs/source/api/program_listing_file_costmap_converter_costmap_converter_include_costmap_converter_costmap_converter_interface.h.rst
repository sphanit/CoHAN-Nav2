
.. _program_listing_file_costmap_converter_costmap_converter_include_costmap_converter_costmap_converter_interface.h:

Program Listing for File costmap_converter_interface.h
======================================================

|exhale_lsh| :ref:`Return to documentation for file <file_costmap_converter_costmap_converter_include_costmap_converter_costmap_converter_interface.h>` (``costmap_converter/costmap_converter/include/costmap_converter/costmap_converter_interface.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2016
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
   
   #ifndef COSTMAP_CONVERTER_INTERFACE_H_
   #define COSTMAP_CONVERTER_INTERFACE_H_
   
   //#include <costmap_2d/costmap_2d_ros.h>
   #include <mutex>
   #include <memory>
   
   #include <rclcpp/rclcpp.hpp>
   #include <rclcpp_lifecycle/lifecycle_node.hpp>
   #include <nav2_costmap_2d/costmap_2d.hpp>
   #include <nav2_costmap_2d/costmap_2d_ros.hpp>
   #include <geometry_msgs/msg/polygon.hpp>
   #include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
   
   namespace costmap_converter
   {
     
   typedef costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr ObstacleArrayPtr;
   typedef costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr ObstacleArrayConstPtr;
   
   typedef std::shared_ptr<std::vector<geometry_msgs::msg::Polygon>> PolygonContainerPtr;
   typedef std::shared_ptr<const std::vector<geometry_msgs::msg::Polygon>> PolygonContainerConstPtr;
     
   
   class BaseCostmapToPolygons
   {
   public: 
     
       virtual void initialize(rclcpp::Node::SharedPtr nh) {
         nh_ = nh;
       }
       
       virtual ~BaseCostmapToPolygons() 
       {
         stopWorker();
       }
   
       
       virtual void setCostmap2D(nav2_costmap_2d::Costmap2D* costmap) = 0;
       
       virtual void updateCostmap2D() = 0;
       
       virtual void compute() = 0;
       
       virtual PolygonContainerConstPtr getPolygons(){return PolygonContainerConstPtr();}
   
       virtual ObstacleArrayConstPtr getObstacles()
       {
         ObstacleArrayPtr obstacles = std::make_shared<costmap_converter_msgs::msg::ObstacleArrayMsg>();
         PolygonContainerConstPtr polygons = getPolygons();
         if (polygons)
         {
           for (const geometry_msgs::msg::Polygon& polygon : *polygons)
           {
             obstacles->obstacles.emplace_back();
             obstacles->obstacles.back().polygon = polygon;
           }
         }
         return obstacles;
       }
   
       virtual void setOdomTopic(const std::string& odom_topic) { (void)odom_topic; }
   
       virtual bool stackedCostmapConversion() {return false;}
   
       void startWorker(rclcpp::Rate::SharedPtr rate, nav2_costmap_2d::Costmap2D* costmap, bool spin_thread = false)
       {
         setCostmap2D(costmap);
         
         if (spin_thread_)
         {
           {
             std::lock_guard<std::mutex> terminate_lock(terminate_mutex_);
             need_to_terminate_ = true;
           }
           spin_thread_->join();
           delete spin_thread_;
         }
         
         if (spin_thread)
         {
           RCLCPP_DEBUG(nh_->get_logger(), "costmap_converter", "Spinning up a thread for the CostmapToPolygons plugin");
           need_to_terminate_ = false;
           
           worker_timer_ = nh_->create_wall_timer(
                       rate->period(),
                       std::bind(&BaseCostmapToPolygons::workerCallback, this));
           spin_thread_ = new std::thread(std::bind(&BaseCostmapToPolygons::spinThread, this));
         }
         else
         {
           worker_timer_ = nh_->create_wall_timer(
                       rate->period(),
                       std::bind(&BaseCostmapToPolygons::workerCallback, this));
           spin_thread_ = nullptr;
         }
       }
       
       void stopWorker()
       {
         if (worker_timer_) worker_timer_->cancel();
         if (spin_thread_)
         {
           {
             std::lock_guard<std::mutex> terminate_lock(terminate_mutex_);
             need_to_terminate_ = true;
           }
           spin_thread_->join();
           delete spin_thread_;
         }
       }
   
   protected:
     
       BaseCostmapToPolygons() : //nh_("~costmap_to_polygons"),
           nh_(nullptr),
           spin_thread_(nullptr), need_to_terminate_(false) {}
       
       void spinThread()
       {
         while (rclcpp::ok())
         {
           {
             std::lock_guard<std::mutex> terminate_lock(terminate_mutex_);
             if (need_to_terminate_)
               break;
             rclcpp::spin_some(nh_);
           }
         }
       }
       
       void workerCallback()
       {
         updateCostmap2D();
         compute();
       }
   
       rclcpp::Logger getLogger() const
       {
           return nh_->get_logger();
       }
   
       rclcpp::Time now() const
       {
           return nh_->now();
       }
       
   private:
     rclcpp::TimerBase::SharedPtr worker_timer_;
     rclcpp::Node::SharedPtr nh_;
     std::thread* spin_thread_;
     std::mutex terminate_mutex_;
     bool need_to_terminate_;
   };    
   
   
   class BaseCostmapToDynamicObstacles : public BaseCostmapToPolygons
   {
   public:
   
     void loadStaticCostmapConverterPlugin(const std::string& plugin_name, rclcpp::Node::SharedPtr nh_parent)
     {
       try
       {
         static_costmap_converter_ = static_converter_loader_.createSharedInstance(plugin_name);
   
         if(std::dynamic_pointer_cast<BaseCostmapToDynamicObstacles>(static_costmap_converter_))
         {
           throw pluginlib::PluginlibException("The specified plugin for static costmap conversion is a dynamic plugin. Specify a static plugin.");
         }
   //      std::string raw_plugin_name = static_converter_loader_.getName(plugin_name);
         static_costmap_converter_->initialize(nh_parent);
         setStaticCostmapConverterPlugin(static_costmap_converter_);
         RCLCPP_INFO(getLogger(), "CostmapToDynamicObstacles: underlying costmap conversion plugin for static obstacles %s loaded.", plugin_name);
       }
       catch(const pluginlib::PluginlibException& ex)
       {
         RCLCPP_WARN(getLogger(), "CostmapToDynamicObstacles: The specified costmap converter plugin cannot be loaded. "
                                  "Continuing without subsequent conversion of static obstacles. Error message: %s", ex.what());
         static_costmap_converter_.reset();
       }
     }
   
     void setStaticCostmapConverterPlugin(std::shared_ptr<BaseCostmapToPolygons> static_costmap_converter)
     {
       static_costmap_converter_ = static_costmap_converter;
     }
   
     void setStaticCostmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> static_costmap)
     {
       static_costmap_converter_->setCostmap2D(static_costmap.get());
     }
   
     void convertStaticObstacles()
     {
       static_costmap_converter_->compute();
     }
   
     PolygonContainerConstPtr getStaticPolygons()
     {
       return static_costmap_converter_->getPolygons();
     }
   
     bool stackedCostmapConversion()
     {
       if(static_costmap_converter_)
         return true;
       else
         return false;
     }
   
   protected:
     BaseCostmapToDynamicObstacles() : static_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"), static_costmap_converter_() {}
   
   private:
     pluginlib::ClassLoader<BaseCostmapToPolygons> static_converter_loader_;
     std::shared_ptr<BaseCostmapToPolygons> static_costmap_converter_;
   };
   
   
   }
   
   
   
   #endif // end COSTMAP_CONVERTER_INTERFACE_H_
