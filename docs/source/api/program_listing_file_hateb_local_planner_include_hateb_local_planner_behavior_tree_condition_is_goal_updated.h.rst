
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_behavior_tree_condition_is_goal_updated.h:

Program Listing for File is_goal_updated.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_behavior_tree_condition_is_goal_updated.h>` (``hateb_local_planner/include/hateb_local_planner/behavior_tree/condition/is_goal_updated.h``)

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
   
   #include <hateb_local_planner/behavior_tree/bt_core.h>
   
   namespace hateb_local_planner {
   
   class IsGoalUpdated : public BT::ConditionNode {
    public:
     IsGoalUpdated(const std::string& condition_name, const BT::NodeConfiguration& conf);
   
     IsGoalUpdated() = delete;
   
     ~IsGoalUpdated() override;
   
     BT::NodeStatus tick() override;
   
     static BT::PortsList providedPorts() {
       return {BT::InputPort<bool>("goal_update"),  
               BT::InputPort<bool>("recovery")};    
     }
   
    private:
     std::string name_;  
   };
   };  // namespace hateb_local_planner
