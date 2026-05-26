
.. _program_listing_file_cohan_layers_include_cohan_layers_agent_layer_config.hpp:

Program Listing for File agent_layer_config.hpp
===============================================

|exhale_lsh| :ref:`Return to documentation for file <file_cohan_layers_include_cohan_layers_agent_layer_config.hpp>` (``cohan_layers/include/cohan_layers/agent_layer_config.hpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*******************************************************************************
    * Software License Agreement (MIT License)
    *
    * Copyright (c) 2025 LAAS-CNRS
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
   
   #ifndef AGENT_LAYER_CONFIG_HPP_
   #define AGENT_LAYER_CONFIG_HPP_
   
   #include <cmath>
   #include <rclcpp_lifecycle/lifecycle_node.hpp>
   #include <ros2_helpers/parameters.hpp>
   
   namespace cohan_layers {
   class AgentLayerConfig {
    public:
     AgentLayerConfig() = default;
     ~AgentLayerConfig() = default;
   
     void initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string layer_name) {
       layer_name_ = layer_name;
       param_helper_.initialize(node);
       RCLCPP_INFO(node->get_logger(), "Initializing parameters for layer: %s", layer_name.c_str());
     }
   
     void setupParameterCallback() {
       // Bind all parameters with automatic updates
       bindParameters();
   
       // Set up parameter change callback - parameters are auto-updated by bindings
       param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter>& params) -> bool {
         // Parameters are automatically updated by ParameterHelper bindings
         return true;
       });
   
       // Load initial parameter values
       param_helper_.loadBoundParameters();
     }
   
     bool enabled;         
     double amplitude;     
     double radius;        
     double agent_radius;  
     std::string ns;       
   
    private:
     void bindParameters() {
       // Set default values for parameters BEFORE binding
       enabled = true;
       amplitude = 150.0;
       radius = 1.5;
       agent_radius = 0.3;
       ns = "";
   
       // Bind parameters with layer name prefix for scoping
       param_helper_.bindBoolParam(layer_name_ + ".enabled", enabled, "Whether to apply this plugin or not");
       param_helper_.bindFloatParam(layer_name_ + ".amplitude", amplitude, 0.0, 254.0, "Amplitude of adjustments at peak");
       param_helper_.bindFloatParam(layer_name_ + ".radius", radius, 0.0, 10.0, "Radius of the Gaussian");
       param_helper_.bindFloatParam(layer_name_ + ".agent_radius", agent_radius, 0.0, 10.0, "Radius of the agent");
       param_helper_.bindStringParam(layer_name_ + ".ns", ns, "ROS namespace for topics and services");
     }
   
     parameters::ParameterHelper param_helper_;  
     std::string layer_name_;                    
   };
   
   }  // namespace cohan_layers
   
   #endif  // AGENT_LAYER_CONFIG_HPP_
