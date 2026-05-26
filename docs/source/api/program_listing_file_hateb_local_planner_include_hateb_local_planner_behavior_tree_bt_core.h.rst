
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_behavior_tree_bt_core.h:

Program Listing for File bt_core.h
==================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_behavior_tree_bt_core.h>` (``hateb_local_planner/include/hateb_local_planner/behavior_tree/bt_core.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*******************************************************************************
    * Software License Agreement (MIT License)
    *
    * Copyright (c) 2024-2025 LAAS-CNRS
    *
    * Permission is hereby granted, free of charge, to any person obtaining a copy
    * of this software and associated documentation files (the "Software"), to deal
    * in the Software without restriction, including without limitation the rights
    * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    * copies of the Software, and to permit persons to whom the Software is
    * furnished to do so, subject to the following conditions:
    *
    * The above copyright notice and this permission notice shall be included in
    * all copies or substantial portions of the Software.
    *
    * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    * THE SOFTWARE.
    *
    * Author: Phani Teja Singamaneni
    *********************************************************************************/
   
   #ifndef BT_CORE_HH_
   #define BT_CORE_HH_
   
   #include <rclcpp/rclcpp.hpp>
   #include <rclcpp_lifecycle/lifecycle_node.hpp>
   
   // Use system BehaviorTree.CPP v3 headers (ROS 2 Humble)
   #include "behaviortree_cpp_v3/bt_factory.h"
   
   #define EPS 0.01      // Small epsilon value for floating-point comparisons
   #define DIST_EPS 0.3  // Distance threshold for proximity checks
   
   #ifndef BTPRINT
   #define BTPRINT 0  // Default to disabled if not set by CMake
   #endif
   
   #if BTPRINT
   #define BT_INFO(x, y) std::cout << "BT_INFO: " << x << " -> " << y << std::endl;
   
   #define BT_WARN(x, y)                                           \
     std::cout << "\033[33m";                                      \
     std::cout << "BT_WARNING: " << x << " -> " << y << std::endl; \
     std::cout << "\033[0m";
   
   #define BT_ERROR(x, y)                                        \
     std::cout << "\033[31m";                                    \
     std::cout << "BT_ERROR: " << x << " -> " << y << std::endl; \
     std::cout << "\033[0m";
   
   #else
   #define BT_INFO(x, y)
   #define BT_WARN(x, y)
   #define BT_ERROR(x, y)
   #endif
   
   inline double normalize_angle(double angle_radians) { return angle_radians - (2.0 * M_PI * std::floor((angle_radians + (M_PI)) / (2.0 * M_PI))); }
   
   /*This part of the code is inspired from here: https://github.com/BehaviorTree/BehaviorTree.ROS (bt_action_node.hh)*/
   namespace hateb_local_planner {
   
   class StatefulActionNodeROS : public BT::StatefulActionNode {
    protected:
     StatefulActionNodeROS(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string& name, const BT::NodeConfiguration& conf) : BT::StatefulActionNode(name, conf), node_(node) {}
   
    public:
     // using BaseClass = StatefulActionNodeROS<ActionT>;
     // using ActionType = ActionT;
   
     StatefulActionNodeROS() = delete;
   
     ~StatefulActionNodeROS() override = default;
   
     static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("action_name")}; }
   
     BT::NodeStatus onStart() override = 0;
   
     BT::NodeStatus onRunning() override = 0;
   
     void onHalted() override = 0;
   
    protected:
     rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  // ROS2 node shared pointer for communication
   };
   
   template <class DerivedT>
   static void RegisterStatefulActionNodeROS(BT::BehaviorTreeFactory& factory, const std::string& registration_ID, rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
     BT::NodeBuilder builder = [node](const std::string& name, const BT::NodeConfiguration& config) { return std::make_unique<DerivedT>(node, name, config); };
   
     BT::TreeNodeManifest manifest;
     manifest.type = BT::getType<DerivedT>();
     manifest.ports = DerivedT::providedPorts();
     manifest.registration_ID = registration_ID;
     const auto& basic_ports = StatefulActionNodeROS::providedPorts();
     manifest.ports.insert(basic_ports.begin(), basic_ports.end());
     factory.registerBuilder(manifest, builder);
   };
   
   enum PLAN : std::uint8_t {
     SINGLE_BAND,  
     DUAL_BAND,    
     VELOBS,       
     PASSTHROUGH   
   };
   
   enum PREDICTION : std::uint8_t {
     CONST_VEL,  
     BEHIND,     
     PREDICT,    
     EXTERNAL    
   };
   
   // 'Using' leads to linkage errors
   typedef struct {
     PLAN plan;           
     PREDICTION predict;  
   } ModeInfo;
   
   }  // namespace hateb_local_planner
   
   #endif  // BT_CORE_HH_
