
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_g2o_types_vertex_pose.h:

Program Listing for File vertex_pose.h
======================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_g2o_types_vertex_pose.h>` (``hateb_local_planner/include/hateb_local_planner/g2o_types/vertex_pose.h``)

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
   
   #ifndef VERTEX_POSE_H_
   #define VERTEX_POSE_H_
   
   #include <g2o/config.h>
   #include <g2o/core/base_vertex.h>
   #include <g2o/core/hyper_graph_action.h>
   #include <g2o/stuff/misc.h>
   #include <hateb_local_planner/pose_se2.h>
   
   namespace hateb_local_planner {
   
   class VertexPose : public g2o::BaseVertex<3, PoseSE2> {
    public:
     explicit VertexPose(bool fixed = false) {
       setToOriginImpl();
       setFixed(fixed);
     }
   
     explicit VertexPose(const PoseSE2& pose, bool fixed = false) {
       _estimate = pose;
       setFixed(fixed);
     }
   
     VertexPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed = false) {
       _estimate.position() = position;
       _estimate.theta() = theta;
       setFixed(fixed);
     }
   
     VertexPose(double x, double y, double theta, bool fixed = false) {
       _estimate.x() = x;
       _estimate.y() = y;
       _estimate.theta() = theta;
       setFixed(fixed);
     }
   
     ~VertexPose() override = default;
   
     inline PoseSE2& pose() { return _estimate; }
   
     inline const PoseSE2& pose() const { return _estimate; }
   
     inline Eigen::Vector2d& position() { return _estimate.position(); }
   
     inline const Eigen::Vector2d& position() const { return _estimate.position(); }
   
     inline double& x() { return _estimate.x(); }
   
     inline const double& x() const { return _estimate.x(); }
   
     inline double& y() { return _estimate.y(); }
   
     inline const double& y() const { return _estimate.y(); }
   
     inline double& theta() { return _estimate.theta(); }
   
     inline const double& theta() const { return _estimate.theta(); }
   
     void setToOriginImpl() override { _estimate.setZero(); }
   
     void oplusImpl(const double* update) override { _estimate.plus(update); }
   
     bool read(std::istream& is) override {
       is >> _estimate.x() >> _estimate.y() >> _estimate.theta();
       return true;
     }
   
     bool write(std::ostream& os) const override {
       os << _estimate.x() << " " << _estimate.y() << _estimate.theta();
       return os.good();
     }
   
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   }  // namespace hateb_local_planner
   
   #endif  // VERTEX_POSE_H_
