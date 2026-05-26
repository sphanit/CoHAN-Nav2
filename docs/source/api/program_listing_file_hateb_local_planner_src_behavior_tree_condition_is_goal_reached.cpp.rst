
.. _program_listing_file_hateb_local_planner_src_behavior_tree_condition_is_goal_reached.cpp:

Program Listing for File is_goal_reached.cpp
============================================

|exhale_lsh| :ref:`Return to documentation for file <file_hateb_local_planner_src_behavior_tree_condition_is_goal_reached.cpp>` (``hateb_local_planner/src/behavior_tree/condition/is_goal_reached.cpp``)

.. |exhale_lsh| unicode:: U+021B0 .. UPWARDS ARROW WITH TIP LEFTWARDS

.. code-block:: cpp

   #include <hateb_local_planner/behavior_tree/condition/is_goal_reached.h>
   
   namespace hateb_local_planner {
   
   IsGoalReached::IsGoalReached(const std::string& condition_name, const BT::NodeConfiguration& conf) : BT::ConditionNode(condition_name, conf), goal_reached_(true) {
     // Initialize ROS
     ros::NodeHandle node("~/");
     goal_sub_ = node.subscribe("/move_base/goal", 1, &IsGoalReached::goalReceivedCB, this);
     status_sub_ = node.subscribe("/move_base/result", 1, &IsGoalReached::resultCB, this);
   
     // set the node name
     name_ = condition_name;
   }
   
   IsGoalReached::~IsGoalReached() {
     // ROS_INFO in destructor
     ROS_INFO("Shutting downd the isGoalReached BT Node");
   }
   
   void IsGoalReached::goalReceivedCB(const move_base_msgs::MoveBaseActionGoal& goal_msg) {
     // Update the goal
     if (!goal_reached_) {
       BT_INFO(name_, "Goal updated.")
       updated_ = true;
     }
   
     goal_reached_ = false;
   
     // Update the class property and the blackboard values with new goal
     goal_ = goal_msg.goal.target_pose.pose;
     setOutput("nav_goal", goal_);
   }
   
   // Is it needed if we are checking the goal reaching manually? TODO: Check
   void IsGoalReached::resultCB(const move_base_msgs::MoveBaseActionResult& result_msg) {
     // Update the goal reached flag
     if (static_cast<int>(result_msg.status.status) == 3) {
       goal_reached_ = true;
     }
   }
   
   BT::NodeStatus IsGoalReached::tick() {
     // Check the goal reached condition
     goal_reached_ = goalReachedCheck();
   
     // Goal updated case
     if (updated_) {
       updated_ = false;
       BT_INFO(name_, "Goal updated, restarting the tree!")
       return BT::NodeStatus::SUCCESS;
     }
     // Goal reached case
     if (goal_reached_) {
       BT_INFO(name_, "Goal reached!")
       return BT::NodeStatus::SUCCESS;
     }
   
     BT_INFO(name_, "Goal in progress.")
     return BT::NodeStatus::FAILURE;
   }
   
   bool IsGoalReached::goalReachedCheck() {
     // Return true if goal_reached_ is already true
     if (goal_reached_) {
       return true;
     }
   
     // Read the input message agents_info from the blackboard
     getInput("agents_info", agents_info_);
   
     // Check the goal manually
     double delta_orient = normalize_angle(tf2::getYaw(goal_.orientation) - agents_info_.robot_pose.theta);
     double dg = std::hypot(goal_.position.x - agents_info_.robot_pose.x, goal_.position.y - agents_info_.robot_pose.y);
   
     return (fabs(dg) < EPS && fabs(delta_orient) < EPS);
   }
   
   };  // namespace hateb_local_planner
