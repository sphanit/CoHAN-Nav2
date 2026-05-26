
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_behavior_tree_condition_vel_obs_exit_condition.h:

Program Listing for File vel_obs_exit_condition.h
=================================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_behavior_tree_condition_vel_obs_exit_condition.h>` (``hateb_local_planner/include/hateb_local_planner/behavior_tree/condition/vel_obs_exit_condition.h``)

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
   
   #include <tf2/utils.h>
   
   #include <rclcpp/rclcpp.hpp>
   
   // New
   #include <hateb_local_planner/behavior_tree/bt_core.h>
   
   #include <agent_path_prediction/msg/agents_info.hpp>
   #include <hateb_local_planner/agents_class.hpp>
   #include <mutex>
   
   namespace hateb_local_planner {
   
   class VelObsExitCondition : public BT::ConditionNode {
    public:
     VelObsExitCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);
   
     VelObsExitCondition() = delete;
   
     ~VelObsExitCondition() override;
   
     BT::NodeStatus tick() override;
   
     static BT::PortsList providedPorts() {
       // This action has a single input port called "agents_info"
       return {BT::InputPort<agent_path_prediction::msg::AgentsInfo>("agents_info"), BT::InputPort<std::shared_ptr<hateb_local_planner::Agents>>("agents_ptr"), BT::OutputPort<int>("stuck_agent")};
     }
   
    private:
     bool hasHumanStopped();
   
     // bool isHumanPlaying(); // Add this later
   
     // Blackboard entries
     agent_path_prediction::msg::AgentsInfo agents_info_;       
     std::shared_ptr<hateb_local_planner::Agents> agents_ptr_;  
   
     std::string name_;         
     int nearest_human_id_;     
     int t_stuck_;              
     std::mutex agents_mutex_;  
   };
   };  // namespace hateb_local_planner
