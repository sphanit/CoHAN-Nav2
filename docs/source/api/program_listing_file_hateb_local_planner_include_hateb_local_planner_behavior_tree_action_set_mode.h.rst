
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_behavior_tree_action_set_mode.h:

Program Listing for File set_mode.h
===================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_behavior_tree_action_set_mode.h>` (``hateb_local_planner/include/hateb_local_planner/behavior_tree/action/set_mode.h``)

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
   
   #include <hateb_local_planner/msg/planning_mode.hpp>
   #include <memory>
   #include <string>
   
   namespace hateb_local_planner {
   
   class SetMode : public BT::StatefulActionNode {
    public:
     SetMode(const std::string& name, const BT::NodeConfiguration& config);
   
     SetMode() = delete;
   
     ~SetMode() override;
   
     static BT::PortsList providedPorts() {
       // define the input and output ports
       return {BT::InputPort<std::string>("plan_type"), BT::InputPort<std::string>("predict_type"), BT::OutputPort<ModeInfo>("mode")};
     }
   
     BT::NodeStatus onStart() override;
   
     BT::NodeStatus onRunning() override;
   
     void onHalted() override;
   
    private:
     std::string name_;          
     std::string plan_type_;     
     std::string predict_type_;  
     ModeInfo p_msg_;            
   };
   };  // namespace hateb_local_planner
