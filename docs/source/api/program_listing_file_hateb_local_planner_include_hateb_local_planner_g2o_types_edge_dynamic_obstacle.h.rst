
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_dynamic_obstacle.h:

Program Listing for File edge_dynamic_obstacle.h
================================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_dynamic_obstacle.h>` (``hateb_local_planner/include/hateb_local_planner/g2o_types/edge_dynamic_obstacle.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    *  Copyright (c) 2016,
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
    * Notes:
    * The following class is derived from a class defined by the
    * g2o-framework. g2o is licensed under the terms of the BSD License.
    * Refer to the base class source for detailed licensing information.
    *
    * Author: Christoph Rösmann, Franz Albers
    *********************************************************************/
   
   #ifndef EDGE_DYNAMICOBSTACLE_H
   #define EDGE_DYNAMICOBSTACLE_H
   
   #include <hateb_local_planner/footprint_model.h>
   #include <hateb_local_planner/g2o_types/base_teb_edges.h>
   #include <hateb_local_planner/g2o_types/penalties.h>
   #include <hateb_local_planner/g2o_types/vertex_pose.h>
   #include <hateb_local_planner/g2o_types/vertex_timediff.h>
   #include <hateb_local_planner/obstacles.h>
   
   #include <cohan_msgs/msg/agent_type.hpp>
   #include <hateb_local_planner/hateb_config.hpp>
   
   namespace hateb_local_planner {
   
   class EdgeDynamicObstacle : public BaseTebUnaryEdge<2, const Obstacle*, VertexPose> {
    public:
     EdgeDynamicObstacle() : t_(0) {}
   
     explicit EdgeDynamicObstacle(double t) : t_(t) {}
   
     void computeError() override {
       HATEB_ASSERT_MSG(cfg_ && _measurement, "You must call setHATebConfig() on EdgeDynamicObstacle()");
       const auto* bandpt = static_cast<const VertexPose*>(_vertices[0]);
   
       double dist = 0;
   
       if (type_ == static_cast<int>(cohan_msgs::msg::AgentType::ROBOT)) {
         dist = cfg_->robot_model->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_);
       }
   
       if (type_ == static_cast<int>(cohan_msgs::msg::AgentType::HUMAN)) {
         dist = cfg_->human_model->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_);
       }
   
       _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
       _error[1] = penaltyBoundFromBelow(dist, cfg_->obstacles.dynamic_obstacle_inflation_dist, 0.0);
   
       HATEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeDynamicObstacle::computeError() _error[0]=%f\n", _error[0]);
     }
   
     void setParameters(const HATebConfig& cfg, const Obstacle* obstacle, const int type) {
       cfg_ = &cfg;
       _measurement = obstacle;
       type_ = type;
     }
   
    protected:
     double t_;  
     int type_;
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   }  // namespace hateb_local_planner
   
   #endif
