
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_g2o_types_base_teb_edges.h:

Program Listing for File base_teb_edges.h
=========================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_g2o_types_base_teb_edges.h>` (``hateb_local_planner/include/hateb_local_planner/g2o_types/base_teb_edges.h``)

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
   
   #ifndef _BASE_TEB_EDGES_H_
   #define _BASE_TEB_EDGES_H_
   
   #include <g2o/core/base_binary_edge.h>
   #include <g2o/core/base_multi_edge.h>
   #include <g2o/core/base_unary_edge.h>
   
   #include <cassert>
   #include <hateb_local_planner/hateb_config.hpp>
   #include <rclcpp/logging.hpp>
   
   // ROS2 compatible assertion macro - equivalent to ROS1's HATEB_ASSERT_MSG
   #ifndef NDEBUG
   #define HATEB_ASSERT_MSG(condition, ...)                                    \
     do {                                                                      \
       if (!(condition)) {                                                     \
         RCLCPP_FATAL(rclcpp::get_logger("hateb_local_planner"), __VA_ARGS__); \
         assert(condition);                                                    \
       }                                                                       \
     } while (0)
   #else
   #define HATEB_ASSERT_MSG(condition, ...) ((void)0)
   #endif
   
   namespace hateb_local_planner {
   
   template <int D, typename E, typename VertexXi>
   class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi> {
    public:
     using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
     using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;
   
     ErrorVector& getError() {
       computeError();
       return _error;
     }
   
     virtual bool read(std::istream& is) {
       // TODO generic read
       return true;
     }
   
     virtual bool write(std::ostream& os) const {
       // TODO generic write
       return os.good();
     }
   
     void setHATebConfig(const HATebConfig& cfg) { cfg_ = &cfg; }
   
    protected:
     using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
     using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;
   
     const HATebConfig* cfg_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   template <int D, typename E, typename VertexXi, typename VertexXj>
   class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj> {
    public:
     using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
     using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;
   
     ErrorVector& getError() {
       computeError();
       return _error;
     }
   
     virtual bool read(std::istream& is) {
       // TODO generic read
       return true;
     }
   
     virtual bool write(std::ostream& os) const {
       // TODO generic write
       return os.good();
     }
   
     void setHATebConfig(const HATebConfig& cfg) { cfg_ = &cfg; }
   
    protected:
     using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
     using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;
   
     const HATebConfig* cfg_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   template <int D, typename E>
   class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E> {
    public:
     using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
     using g2o::BaseMultiEdge<D, E>::computeError;
   
     // Overwrites resize() from the parent class
     virtual void resize(size_t size) {
       g2o::BaseMultiEdge<D, E>::resize(size);
   
       for (std::size_t i = 0; i < _vertices.size(); ++i) _vertices[i] = NULL;
     }
   
     ErrorVector& getError() {
       computeError();
       return _error;
     }
   
     virtual bool read(std::istream& is) {
       // TODO generic read
       return true;
     }
   
     virtual bool write(std::ostream& os) const {
       // TODO generic write
       return os.good();
     }
   
     void setHATebConfig(const HATebConfig& cfg) { cfg_ = &cfg; }
   
    protected:
     using g2o::BaseMultiEdge<D, E>::_error;
     using g2o::BaseMultiEdge<D, E>::_vertices;
   
     const HATebConfig* cfg_;  
   
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   };
   
   }  // namespace hateb_local_planner
   
   #endif
