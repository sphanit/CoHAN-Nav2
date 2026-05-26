
.. _program_listing_file_cohan_layers_include_cohan_layers_static_agent_layer.hpp:

Program Listing for File static_agent_layer.hpp
===============================================

|exhale_lsh| :ref:`Return to documentation for file <file_cohan_layers_include_cohan_layers_static_agent_layer.hpp>` (``cohan_layers/include/cohan_layers/static_agent_layer.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*******************************************************************************
    * Software License Agreement (MIT License)
    *
    * Copyright (c) 2020-2025 LAAS-CNRS
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
   
   #ifndef STATIC_AGENT_LAYER_H
   #define STATIC_AGENT_LAYER_H
   #include <cohan_layers/agent_layer.hpp>
   #include <rcl_interfaces/msg/set_parameters_result.hpp>
   #include <rclcpp/rclcpp.hpp>
   #include <vector>
   
   namespace cohan_layers {
   class StaticAgentLayer : public AgentLayer {
    public:
     StaticAgentLayer() { layered_costmap_ = nullptr; }
   
     void onInitialize() override;
   
     void updateBoundsFromAgents(double* min_x, double* min_y, double* max_x, double* max_y) override;
   
     void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
   
     void matchSize() override {}
   
     void reset() override { current_ = false; }
   
     bool isClearable() override { return true; }
   };
   }  // namespace cohan_layers
   
   #endif  // STATIC_AGENT_LAYER_H
