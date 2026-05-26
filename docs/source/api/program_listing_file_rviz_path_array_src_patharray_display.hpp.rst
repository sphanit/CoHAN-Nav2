
.. _program_listing_file_rviz_path_array_src_patharray_display.hpp:

Program Listing for File patharray_display.hpp
==============================================

|exhale_lsh| :ref:`Return to documentation for file <file_rviz_path_array_src_patharray_display.hpp>` (``rviz_path_array/src/patharray_display.hpp``)

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
   
   #ifndef AGENT_PATHARRAY_DISPLAY_HPP
   #define AGENT_PATHARRAY_DISPLAY_HPP
   
   #include <cohan_msgs/msg/agent_path.hpp>
   #include <cohan_msgs/msg/agent_path_array.hpp>
   #include <rviz_default_plugins/displays/marker/marker_common.hpp>
   #include <rviz_rendering/objects/arrow.hpp>
   #include <rviz_rendering/objects/axes.hpp>
   #include <rviz_rendering/objects/billboard_line.hpp>
   
   #include "rviz_common/message_filter_display.hpp"
   
   namespace Ogre {
   class ManualObject;
   }
   
   namespace rviz_common {
   namespace properties {
   class ColorProperty;
   class FloatProperty;
   class IntProperty;
   class EnumProperty;
   class VectorProperty;
   }  // namespace properties
   }  // namespace rviz_common
   
   namespace rviz_path_array {
   
   class AgentPathArrayDisplay;
   class AgentPathArrayDisplay : public rviz_common::MessageFilterDisplay<cohan_msgs::msg::AgentPathArray> {
     Q_OBJECT
    public:
     AgentPathArrayDisplay();
     ~AgentPathArrayDisplay() override;
   
     void reset() override;
   
    protected:
     void onInitialize() override;
   
     void processMessage(cohan_msgs::msg::AgentPathArray::ConstSharedPtr msg) override;
   
    private Q_SLOTS:
     void updateBufferLength();
     void updateStyle();
     void updateLineWidth();
     void updateOffset();
     void updatePoseStyle();
     void updatePoseAxisGeometry();
     void updatePoseArrowColor();
     void updatePoseArrowGeometry();
   
    private:
     void destroyObjects();
     void allocateArrowVector(std::vector<rviz_rendering::Arrow*>& arrow_vect, int num);
     void allocateAxesVector(std::vector<rviz_rendering::Axes*>& axes_vect, int num);
     void destroyPoseAxesChain();
     void destroyPoseArrowChain();
   
     std::vector<Ogre::ManualObject*> manual_objects_;
     std::vector<rviz_rendering::BillboardLine*> billboard_lines_;
     std::vector<std::vector<rviz_rendering::Axes*> > axes_chain_;
     std::vector<std::vector<rviz_rendering::Arrow*> > arrow_chain_;
   
     rviz_common::properties::EnumProperty* style_property_;
     rviz_common::properties::ColorProperty* color_property_;
     rviz_common::properties::FloatProperty* alpha_property_;
     rviz_common::properties::FloatProperty* line_width_property_;
     rviz_common::properties::IntProperty* buffer_length_property_;
     rviz_common::properties::VectorProperty* offset_property_;
   
     enum LineStyle { LINES, BILLBOARDS };
   
     // pose marker property
     rviz_common::properties::EnumProperty* pose_style_property_;
     rviz_common::properties::FloatProperty* pose_axes_length_property_;
     rviz_common::properties::FloatProperty* pose_axes_radius_property_;
     rviz_common::properties::ColorProperty* pose_arrow_color_property_;
     rviz_common::properties::FloatProperty* pose_arrow_shaft_length_property_;
     rviz_common::properties::FloatProperty* pose_arrow_head_length_property_;
     rviz_common::properties::FloatProperty* pose_arrow_shaft_diameter_property_;
     rviz_common::properties::FloatProperty* pose_arrow_head_diameter_property_;
   
     enum PoseStyle {
       NONE,
       AXES,
       ARROWS,
     };
   };
   
   }  // namespace rviz_path_array
   
   #endif /* AGENT_PATHARRAY_DISPLAY_HPP */
