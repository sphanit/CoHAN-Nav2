
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_behavior_tree_condition_passthrough_condition.h:

Program Listing for File passthrough_condition.h
================================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_behavior_tree_condition_passthrough_condition.h>` (``hateb_local_planner/include/hateb_local_planner/behavior_tree/condition/passthrough_condition.h``)

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
   #include <cohan_msgs/msg/passage_type.hpp>
   #include <hateb_local_planner/agents_class.hpp>
   
   namespace hateb_local_planner {
   
   class PassThroughCondition : public BT::ConditionNode {
    public:
     PassThroughCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);
   
     PassThroughCondition() = delete;
   
     ~PassThroughCondition() override;
   
     BT::NodeStatus tick() override;
   
     static BT::PortsList providedPorts() {
       return {BT::InputPort<int8_t>("passage_type")};  
     }
   
    private:
     int8_t psg_type_;  
   
     std::string name_;  
   };
   };  // namespace hateb_local_planner
