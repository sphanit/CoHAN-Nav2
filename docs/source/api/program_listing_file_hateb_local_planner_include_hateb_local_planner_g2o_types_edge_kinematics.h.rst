
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_kinematics.h:

Program Listing for File edge_kinematics.h
==========================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_kinematics.h>` (``hateb_local_planner/include/hateb_local_planner/g2o_types/edge_kinematics.h``)

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
   
   #ifndef _EDGE_KINEMATICS_H
   #define _EDGE_KINEMATICS_H
   
   #include <hateb_local_planner/g2o_types/base_teb_edges.h>
   #include <hateb_local_planner/g2o_types/penalties.h>
   #include <hateb_local_planner/g2o_types/vertex_pose.h>
   
   #include <cmath>
   #include <hateb_local_planner/hateb_config.hpp>
   
   namespace hateb_local_planner {
   
   class EdgeKinematicsDiffDrive : public BaseTebBinaryEdge<2, double, VertexPose, VertexPose> {
    public:
     EdgeKinematicsDiffDrive() { this->setMeasurement(0.); }
   
     void computeError() override {
       HATEB_ASSERT_MSG(cfg_, "You must call setHATebConfig on EdgeKinematicsDiffDrive()");
       const auto* conf1 = static_cast<const VertexPose*>(_vertices[0]);
       const auto* conf2 = static_cast<const VertexPose*>(_vertices[1]);
   
       Eigen::Vector2d delta_s = conf2->position() - conf1->position();
   
       // non holonomic constraint
       _error[0] = fabs(((cos(conf1->theta()) + cos(conf2->theta())) * delta_s[1]) - ((sin(conf1->theta()) + sin(conf2->theta())) * delta_s[0]));
   
       // positive-drive-direction constraint
       Eigen::Vector2d angle_vec(cos(conf1->theta()), sin(conf1->theta()));
       _error[1] = penaltyBoundFromBelow(delta_s.dot(angle_vec), 0, 0);
       // epsilon=0, otherwise it pushes the first bandpoints away from start
   
       HATEB_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeKinematicsDiffDrive::computeError() _error[0]=%f _error[1]=%f\n", _error[0], _error[1]);
     }
   
   #ifdef USE_ANALYTIC_JACOBI
   #if 1
     void linearizeOplus() override {
       HATEB_ASSERT_MSG(cfg_, "You must call setHATebConfig on EdgeKinematicsDiffDrive()");
       const auto* conf1 = static_cast<const VertexPose*>(_vertices[0]);
       const auto* conf2 = static_cast<const VertexPose*>(_vertices[1]);
   
       Eigen::Vector2d delta_s = conf2->position() - conf1->position();
   
       double cos1 = cos(conf1->theta());
       double cos2 = cos(conf2->theta());
       double sin1 = sin(conf1->theta());
       double sin2 = sin(conf2->theta());
       double aux1 = sin1 + sin2;
       double aux2 = cos1 + cos2;
   
       double dd_error_1 = delta_s[0] * cos1;
       double dd_error_2 = delta_s[1] * sin1;
       double dd_dev = penaltyBoundFromBelowDerivative(dd_error_1 + dd_error_2, 0, 0);
   
       double dev_nh_abs = g2o::sign(((cos(conf1->theta()) + cos(conf2->theta())) * delta_s[1]) - ((sin(conf1->theta()) + sin(conf2->theta())) * delta_s[0]));
   
       // conf1
       _jacobianOplusXi(0, 0) = aux1 * dev_nh_abs;                                  // nh x1
       _jacobianOplusXi(0, 1) = -aux2 * dev_nh_abs;                                 // nh y1
       _jacobianOplusXi(1, 0) = -cos1 * dd_dev;                                     // drive-dir x1
       _jacobianOplusXi(1, 1) = -sin1 * dd_dev;                                     // drive-dir y1
       _jacobianOplusXi(0, 2) = (-dd_error_2 - dd_error_1) * dev_nh_abs;            // nh angle
       _jacobianOplusXi(1, 2) = (-sin1 * delta_s[0] + cos1 * delta_s[1]) * dd_dev;  // drive-dir angle1
   
       // conf2
       _jacobianOplusXj(0, 0) = -aux1 * dev_nh_abs;                                     // nh x2
       _jacobianOplusXj(0, 1) = aux2 * dev_nh_abs;                                      // nh y2
       _jacobianOplusXj(1, 0) = cos1 * dd_dev;                                          // drive-dir x2
       _jacobianOplusXj(1, 1) = sin1 * dd_dev;                                          // drive-dir y2
       _jacobianOplusXj(0, 2) = (-sin2 * delta_s[1] - cos2 * delta_s[0]) * dev_nh_abs;  // nh angle
       _jacobianOplusXj(1, 2) = 0;                                                      // drive-dir angle1
     }
   #endif
   #endif
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   class EdgeKinematicsCarlike : public BaseTebBinaryEdge<2, double, VertexPose, VertexPose> {
    public:
     EdgeKinematicsCarlike() { this->setMeasurement(0.); }
   
     void computeError() override {
       HATEB_ASSERT_MSG(cfg_, "You must call setHATebConfig on EdgeKinematicsCarlike()");
       const auto* conf1 = static_cast<const VertexPose*>(_vertices[0]);
       const auto* conf2 = static_cast<const VertexPose*>(_vertices[1]);
   
       Eigen::Vector2d delta_s = conf2->position() - conf1->position();
   
       // non holonomic constraint
       _error[0] = fabs(((cos(conf1->theta()) + cos(conf2->theta())) * delta_s[1]) - ((sin(conf1->theta()) + sin(conf2->theta())) * delta_s[0]));
   
       // limit minimum turning radius
       double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
       if (angle_diff == 0)
         _error[1] = 0;                             // straight line motion
       else if (cfg_->trajectory.exact_arc_length)  // use exact computation of the radius
         _error[1] = penaltyBoundFromBelow(fabs(delta_s.norm() / (2 * sin(angle_diff / 2))), cfg_->robot.min_turning_radius, 0.0);
       else
         _error[1] = penaltyBoundFromBelow(delta_s.norm() / fabs(angle_diff), cfg_->robot.min_turning_radius, 0.0);
       // This edge is not affected by the epsilon parameter, the user might add an exra margin to the min_turning_radius parameter.
   
       HATEB_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeKinematicsCarlike::computeError() _error[0]=%f _error[1]=%f\n", _error[0], _error[1]);
     }
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   }  // namespace hateb_local_planner
   
   #endif
