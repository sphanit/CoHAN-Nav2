
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_prefer_rotdir.h:

Program Listing for File edge_prefer_rotdir.h
=============================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_prefer_rotdir.h>` (``hateb_local_planner/include/hateb_local_planner/g2o_types/edge_prefer_rotdir.h``)

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
    * Author: Christoph Rösmann
    *********************************************************************/
   #ifndef EDGE_PREFER_ROTDIR_H_
   #define EDGE_PREFER_ROTDIR_H_
   
   #include <hateb_local_planner/g2o_types/base_teb_edges.h>
   #include <hateb_local_planner/g2o_types/penalties.h>
   #include <hateb_local_planner/g2o_types/vertex_pose.h>
   
   // #include "g2o/core/base_unary_edge.h"
   
   namespace hateb_local_planner {
   
   class EdgePreferRotDir : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
    public:
     EdgePreferRotDir() { _measurement = 1; }
   
     void computeError() override {
       const auto* conf1 = static_cast<const VertexPose*>(_vertices[0]);
       const auto* conf2 = static_cast<const VertexPose*>(_vertices[1]);
   
       _error[0] = penaltyBoundFromBelow(_measurement * g2o::normalize_theta(conf2->theta() - conf1->theta()), 0, 0);
   
       HATEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgePreferRotDir::computeError() _error[0]=%f\n", _error[0]);
     }
   
     void setRotDir(double dir) { _measurement = dir; }
   
     void preferRight() { _measurement = -1; }
   
     void preferLeft() { _measurement = 1; }
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   }  // namespace hateb_local_planner
   
   #endif
