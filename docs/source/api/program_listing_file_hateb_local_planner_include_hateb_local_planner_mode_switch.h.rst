
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_mode_switch.h:

Program Listing for File mode_switch.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_mode_switch.h>` (``hateb_local_planner/include/hateb_local_planner/mode_switch.h``)

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
   
   #ifndef MODE_SWITCH_HH_
   #define MODE_SWITCH_HH_
   
   #include <hateb_local_planner/agents_class.hpp>
   #include <rclcpp/rclcpp.hpp>
   
   // Use system BehaviorTree.CPP v3 (ROS 2 Humble)
   #include "behaviortree_cpp_v3/bt_factory.h"
   
   // Messages
   #include <action_msgs/msg/goal_status_array.hpp>
   #include <agent_path_prediction/msg/agents_info.hpp>
   #include <cohan_msgs/msg/passage_type.hpp>
   #include <geometry_msgs/msg/pose.hpp>
   #include <geometry_msgs/msg/pose_stamped.hpp>
   #include <hateb_local_planner/msg/planning_mode.hpp>
   #include <nav2_msgs/action/navigate_to_pose.hpp>
   #include <nav_msgs/msg/path.hpp>
   #include <std_srvs/srv/trigger.hpp>
   
   // BT Nodes
   #include <hateb_local_planner/behavior_tree/action/set_mode.h>
   #include <hateb_local_planner/behavior_tree/bt_core.h>
   #include <hateb_local_planner/behavior_tree/condition/dual_band_exit_condition.h>
   #include <hateb_local_planner/behavior_tree/condition/is_goal_updated.h>
   #include <hateb_local_planner/behavior_tree/condition/passthrough_condition.h>
   #include <hateb_local_planner/behavior_tree/condition/single_band_exit_condition.h>
   #include <hateb_local_planner/behavior_tree/condition/vel_obs_exit_condition.h>
   
   // Stdlib
   #include <mutex>
   
   // All topics are good here
   // #define AGENTS_INFO_SUB "/agents_info"
   // #define PLAN_SUB "/plan"
   // #define RESULT_SUB "/navigate_to_pose/_action/status"
   // #define PASSAGE_SUB "/invisible_humans_detection/passage"
   
   namespace hateb_local_planner {
   
   class ModeSwitch {
    public:
     ModeSwitch();
   
     ~ModeSwitch() = default;
   
     void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::shared_ptr<HATebConfig> cfg, std::shared_ptr<hateb_local_planner::Agents>& agents_ptr);
   
     BT::NodeStatus tickBT();
   
     void resetBT();
   
     BT::Tree* BTree() { return &bhv_tree_; }
   
     hateb_local_planner::msg::PlanningMode tickAndGetMode();
   
     static void setBTDebugEnabled(bool enabled);
   
    private:
     void registerNodes();
   
     void agentsInfoCB(const agent_path_prediction::msg::AgentsInfo::SharedPtr info_msg);
   
     void planCB(const nav_msgs::msg::Path::SharedPtr plan_msg);
   
     void resultNavigateToPoseCB(const action_msgs::msg::GoalStatusArray::SharedPtr result_msg);
   
     void passageCB(const cohan_msgs::msg::PassageType::SharedPtr passage_msg);
   
     void updateMode(int duration = 0);
   
     void printTreeStatus(const BT::TreeNode* node, int level = 0) {
       std::string indent(level * 2, ' ');
       std::cout << indent << node->name() << ": " << toStr(node->status()) << std::endl;
   
       if (auto control = dynamic_cast<const BT::ControlNode*>(node)) {
         for (unsigned i = 0; i < control->childrenCount(); ++i) {
           printTreeStatus(control->child(i), level + 1);
         }
       }
     }
   
     // Status flags
     bool goal_reached_;  
     bool initialized_;   
     bool goal_update_;   
   
     // ROS communication members
     rclcpp_lifecycle::LifecycleNode::SharedPtr node_;                                          
     rclcpp::Subscription<agent_path_prediction::msg::AgentsInfo>::SharedPtr agents_info_sub_;  
     rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;                            
     rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr result_sub_;            
     rclcpp::Subscription<cohan_msgs::msg::PassageType>::SharedPtr passage_detect_sub_;         
     rclcpp::Publisher<hateb_local_planner::msg::PlanningMode>::SharedPtr planning_mode_pub_;   
   
     // State information
     geometry_msgs::msg::PoseStamped goal_;                
     agent_path_prediction::msg::AgentsInfo agents_info_;  
     action_msgs::msg::GoalStatusArray result_msg_;        
   
     // Behavior Tree components
     BT::BehaviorTreeFactory bhv_factory_;  
     BT::Tree bhv_tree_;                    
   
     std::mutex pub_mutex_;  
   
     std::string name_;                                  
     hateb_local_planner::msg::PlanningMode plan_mode_;  
     ModeInfo mode_info_;                                
   
     // Params for namespace and subscription topics
     std::string ns_;                     
     std::string agents_info_sub_topic_;  
     std::string plan_sub_topic_;         
     std::string result_sub_topic_;       
     std::string passage_sub_topic_;      
   
     std::shared_ptr<HATebConfig> cfg_;
   };
   
   }  // namespace hateb_local_planner
   #endif  // MODE_SWITCH_HH_
