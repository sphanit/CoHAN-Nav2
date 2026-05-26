
.. _program_listing_file_agent_path_prediction_include_agent_path_prediction_predict_goal.h:

Program Listing for File predict_goal.h
=======================================

|exhale_lsh| :ref:`Return to documentation for file <file_agent_path_prediction_include_agent_path_prediction_predict_goal.h>` (``agent_path_prediction/include/agent_path_prediction/predict_goal.h``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   /*********************************************************************
    *
    * Software License Agreement (BSD License)
    *
    * Copyright (c) 2025 LAAS/CNRS
    * All rights reserved.
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
    *  Author: Phani Teja Singamaneni
    *********************************************************************/
   
   #ifndef PREDICT_GOAL_HH_
   #define PREDICT_GOAL_HH_
   #include <cmath>
   #include <eigen3/Eigen/Core>
   #include <map>
   #include <string>
   #include <utility>
   #include <vector>
   #define COV 0.2
   
   namespace agents {
   
   class Gaussian {
    public:
     Gaussian(double mean, double cov) {
       this->mean_ = mean;
       this->cov_ = cov;
       this->inv_cov_ = 1.0 / cov;
       this->norm_const_ = 1.0 / std::sqrt(2 * M_PI * cov);  // Normalizing constant
     }
   
     double pdf(double x) const {
       double diff = x - mean_;
       return norm_const_ * std::exp(-0.5 * diff * diff * inv_cov_);
     }
   
    private:
     double mean_;        // Mean of the Gaussian distribution
     double cov_;         // Covariance of the distribution
     double inv_cov_;     // Inverse of covariance for faster computation
     double norm_const_;  // Normalization constant for PDF
   };
   
   class BayesianGoalPrediction {
    public:
     using Trajectory = std::vector<Eigen::Vector2d>;
   
     BayesianGoalPrediction() : nd_(0, COV) {}
   
     ~BayesianGoalPrediction() = default;
   
     void initialize(const std::map<std::string, Eigen::Vector2d> &goals, int window_size) {
       for (const auto &goal : goals) {
         goal_names_.push_back(goal.first);
         goals_.push_back(goal.second);
       }
       window_size_ = window_size;
     }
   
     std::string predictGoal(int id, Eigen::Vector2d &xy) {
       std::string goal = "None";
   
       addPosition(id, xy);
       if (agents_trajs_[id].size() < window_size_) {
         return goal;
       }
   
       getProbabilities(id);
       int max_prob = std::max_element(agent_probs_[id].begin(), agent_probs_[id].end()) - agent_probs_[id].begin();
       goal = goal_names_[max_prob];
       return goal;
     }
   
    private:
     void addPosition(int id, Eigen::Vector2d &xy) {
       if (agents_trajs_[id].size() > window_size_) {
         agents_trajs_[id].erase(agents_trajs_[id].begin());
       }
       agents_trajs_[id].push_back(xy);
     }
   
     void getProbabilities(int id) {
       std::vector<double> probs;  // Temporary storage for probabilities
       std::vector<double> dists;  // Storage for distances to goals
   
       if (goal_priors_[id].empty()) {
         double prior = 1.0 / static_cast<double>(goals_.size());
         for (int i = 0; i < goals_.size(); i++) {
           goal_priors_[id].push_back(prior);
         }
       }
   
       auto trajectory = agents_trajs_[id];
       int goal_id = 0;
       double probs_sum = 0;
       bool still = false;
   
       for (auto &goal : goals_) {
         double probability = 1.0 / goals_.size();
         int n = trajectory.size();
         for (int i = 1; i < n; i++) {
           Eigen::Vector2d heading = trajectory[i] - trajectory[i - 1];
           if (heading.norm() == 0) {
             still = true;
             break;
           }
   
           Eigen::Vector2d goal_vec = goal - trajectory[i];
           if (i == n - 1) {
             dists.push_back(goal_vec.norm());
           }
   
           double phi = 0;
           if (heading.norm() != 0) {
             phi = std::acos(heading.dot(goal_vec) / (heading.norm() * goal_vec.norm()));
           }
   
           double g = std::exp((i - n) / 0.5);
           probability *= std::pow(nd_.pdf(phi), g);  // Combined proability for the entire trajectory
         }
   
         if (!still) {
           double goal_probability = goal_priors_[id][goal_id];
           goal_probability *= probability;
           probs_sum += goal_probability;
           probs.push_back(goal_probability);
         }
   
         goal_id++;
       }
   
       if (!still) {
         for (auto &prob : probs) {
           prob = prob / probs_sum;
           double prior = prob;
         }
         agent_probs_[id] = probs;
       }
     }
   
     Gaussian nd_;                                     
     std::vector<Eigen::Vector2d> goals_;              
     std::vector<std::string> goal_names_;             
     std::map<int, Trajectory> agents_trajs_;          
     std::map<int, std::vector<double>> goal_priors_;  
     std::map<int, std::vector<double>> agent_probs_;  
     int window_size_;                                 
   };
   }  // namespace agents
   #endif
