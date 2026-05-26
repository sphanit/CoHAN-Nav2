
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_g2o_types_penalties.h:

Program Listing for File penalties.h
====================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_g2o_types_penalties.h>` (``hateb_local_planner/include/hateb_local_planner/g2o_types/penalties.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    * Modified by Phani Teja Singamaneni in 2021
    * Additional changes licensed under the MIT License. See LICENSE file.
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
    * Author: Christoph Rösmann
    *********************************************************************/
   
   #ifndef PENALTIES_H
   #define PENALTIES_H
   
   #include <g2o/stuff/misc.h>
   
   #include <Eigen/Core>
   #include <cmath>
   
   namespace hateb_local_planner {
   
   inline double penaltyBoundToInterval(const double& var, const double& a, const double& epsilon) {
     if (var < -a + epsilon) {
       return (-var - (a - epsilon));
     }
     if (var <= a - epsilon) {
       return 0.;
     }
     return (var - (a - epsilon));
   }
   
   inline double penaltyBoundToInterval(const double& var, const double& a, const double& b, const double& epsilon) {
     if (var < a + epsilon) {
       return (-var + (a + epsilon));
     }
     if (var <= b - epsilon) {
       return 0.;
     }
     return (var - (b - epsilon));
   }
   
   inline double penaltyBoundFromBelow(const double& var, const double& a, const double& epsilon) {
     if (var >= a + epsilon) {
       return 0.0;
     }
     return (-var + (a + epsilon));
   }
   
   inline double penaltyBoundFromAbove(const double& var, const double& a, const double& epsilon) {
     if (var <= (a - epsilon)) {
       return 0.0;
     }
     return (var - (a - epsilon));
   }
   
   inline double penaltyBoundToIntervalDerivative(const double& var, const double& a, const double& epsilon) {
     if (var < -a + epsilon) {
       return -1;
     }
     if (var <= a - epsilon) {
       return 0.;
     }
     return 1;
   }
   
   inline double penaltyBoundToIntervalDerivative(const double& var, const double& a, const double& b, const double& epsilon) {
     if (var < a + epsilon) {
       return -1;
     }
     if (var <= b - epsilon) {
       return 0.;
     }
     return 1;
   }
   
   inline double penaltyBoundFromBelowDerivative(const double& var, const double& a, const double& epsilon) {
     if (var >= a + epsilon) {
       return 0.;
     }
     return -1;
   }
   
   inline double penaltyBoundFromBelowExp(const double& var, const double& a, const double& epsilon, const double& mul) {
     if (var >= a + epsilon) {
       return 0.0;
     }
     if (var < 0.0) {
       return (-var + (a + epsilon));
     }
     return std::max(((a + epsilon - var) / (mul * var + 1.0)),
                     0.00001);  // for numerical stability
   }
   
   inline double penaltyBoundFromBelowNonLinear(const double& var, const double& a, const double& epsilon) {
     if (var >= a + epsilon) {
       return 0.0;
     }
     if (var < 0.0) {
       return (-var + (a + epsilon));
     }
     return std::max(((a + epsilon - var) / (var + 0.1)),
                     0.00001);  // for numerical stability
   }
   
   inline double penaltyBoundFromBelowQuad(const double& var, const double& a, const double& epsilon) {
     if (var >= a + epsilon) {
       return 0.0;
     }
   
     return std::pow((a + epsilon - var), 2) + (a + epsilon - var);
   }
   
   }  // namespace hateb_local_planner
   
   #endif  // PENALTIES_H
