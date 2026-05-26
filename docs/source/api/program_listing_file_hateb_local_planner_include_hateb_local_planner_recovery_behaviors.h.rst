
.. _program_listing_file_hateb_local_planner_include_hateb_local_planner_recovery_behaviors.h:

Program Listing for File recovery_behaviors.h
=============================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_include_hateb_local_planner_recovery_behaviors.h>` (``hateb_local_planner/include/hateb_local_planner/recovery_behaviors.h``)

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
    * Minor Modifications by: Phani Teja Singamaneni
    *********************************************************************/
   
   #ifndef RECOVERY_BEHAVIORS_H__
   #define RECOVERY_BEHAVIORS_H__
   
   #include <boost/circular_buffer.hpp>
   #include <geometry_msgs/msg/twist.hpp>
   #include <rclcpp/rclcpp.hpp>
   
   namespace hateb_local_planner {
   
   class FailureDetector {
    public:
     FailureDetector() = default;
   
     ~FailureDetector() = default;
   
     void setBufferLength(int length) { buffer_.set_capacity(length); }
   
     void update(const geometry_msgs::msg::Twist& twist, double v_max, double v_backwards_max, double omega_max, double v_eps, double omega_eps);
   
     bool isOscillating() const;
   
     void clear();
   
    protected:
     struct VelMeasurement {
       double v = 0;
       double omega = 0;
     };
   
     bool detect(double v_eps, double omega_eps);
   
    private:
     boost::circular_buffer<VelMeasurement> buffer_;  
     bool oscillating_ = false;                       
   };
   
   }  // namespace hateb_local_planner
   
   #endif /* RECOVERY_BEHAVIORS_H__ */
