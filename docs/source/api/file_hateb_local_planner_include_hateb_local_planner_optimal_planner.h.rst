
.. _file_hateb_local_planner_include_hateb_local_planner_optimal_planner.h:

File optimal_planner.h
======================


Definition (``hateb_local_planner/include/hateb_local_planner/optimal_planner.h``)
----------------------------------------------------------------------------------


.. toctree::
   :maxdepth: 1

   program_listing_file_hateb_local_planner_include_hateb_local_planner_optimal_planner.h.rst





Includes
--------


- ``algorithm``

- ``climits``

- ``cmath``

- ``cohan_msgs/msg/agent_type.hpp``

- ``cohan_msgs/msg/trajectory.hpp``

- ``g2o/core/block_solver.h``

- ``g2o/core/factory.h``

- ``g2o/core/optimization_algorithm_gauss_newton.h``

- ``g2o/core/optimization_algorithm_levenberg.h``

- ``g2o/core/sparse_optimizer.h``

- ``g2o/solvers/cholmod/linear_solver_cholmod.h``

- ``g2o/solvers/csparse/linear_solver_csparse.h``

- ``geometry_msgs/msg/point.hpp``

- ``geometry_msgs/msg/pose.hpp``

- ``geometry_msgs/msg/pose_stamped.hpp``

- ``geometry_msgs/msg/twist.hpp``

- ``hateb_local_planner/footprint_model.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_footprint_model.h`)

- ``hateb_local_planner/g2o_types/edge_acceleration.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_acceleration.h`)

- ``hateb_local_planner/g2o_types/edge_agent_agent_safety.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_agent_agent_safety.h`)

- ``hateb_local_planner/g2o_types/edge_agent_robot_rel_velocity.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_agent_robot_rel_velocity.h`)

- ``hateb_local_planner/g2o_types/edge_agent_robot_safety.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_agent_robot_safety.h`)

- ``hateb_local_planner/g2o_types/edge_agent_robot_visibility.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_agent_robot_visibility.h`)

- ``hateb_local_planner/g2o_types/edge_dynamic_obstacle.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_dynamic_obstacle.h`)

- ``hateb_local_planner/g2o_types/edge_invisible_human.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_invisible_human.h`)

- ``hateb_local_planner/g2o_types/edge_kinematics.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_kinematics.h`)

- ``hateb_local_planner/g2o_types/edge_obstacle.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_obstacle.h`)

- ``hateb_local_planner/g2o_types/edge_prefer_rotdir.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_prefer_rotdir.h`)

- ``hateb_local_planner/g2o_types/edge_shortest_path.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_shortest_path.h`)

- ``hateb_local_planner/g2o_types/edge_static_agent_visibility.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_static_agent_visibility.h`)

- ``hateb_local_planner/g2o_types/edge_time_optimal.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_time_optimal.h`)

- ``hateb_local_planner/g2o_types/edge_velocity.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_velocity.h`)

- ``hateb_local_planner/g2o_types/edge_via_point.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_g2o_types_edge_via_point.h`)

- ``hateb_local_planner/hateb_config.hpp`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_hateb_config.hpp`)

- ``hateb_local_planner/misc.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_misc.h`)

- ``hateb_local_planner/planner_interface.h`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_planner_interface.h`)

- ``hateb_local_planner/timed_elastic_band.hpp`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_timed_elastic_band.hpp`)

- ``hateb_local_planner/visualization.hpp`` (:ref:`file_hateb_local_planner_include_hateb_local_planner_visualization.hpp`)

- ``limits``

- ``map`` (:ref:`file_costmap_converter_costmap_converter_src_costmap_to_dynamic_obstacles_background_subtractor.cpp`)

- ``memory``

- ``mutex``

- ``nav_msgs/msg/odometry.hpp``

- ``nav_msgs/msg/path.hpp``

- ``rclcpp/rclcpp.hpp``

- ``rclcpp_lifecycle/lifecycle_node.hpp``

- ``ros2_helpers/utils.hpp`` (:ref:`file_ros2_helpers_include_ros2_helpers_utils.hpp`)

- ``utility``



Included By
-----------


- :ref:`file_hateb_local_planner_include_hateb_local_planner_hateb_local_planner_ros.h`

- :ref:`file_hateb_local_planner_src_optimal_planner.cpp`

- :ref:`file_hateb_local_planner_src_visualization.cpp`




Namespaces
----------


- :ref:`namespace_hateb_local_planner`


Classes
-------


- :ref:`exhale_class_classhateb__local__planner_1_1HATebOptimalPlanner`


Typedefs
--------


- :ref:`exhale_typedef_namespacehateb__local__planner_1aa40dd99696de24d8ac04feb443e22c6d`

- :ref:`exhale_typedef_namespacehateb__local__planner_1a80c55c90860d688747fb08b15a6e80d9`

- :ref:`exhale_typedef_namespacehateb__local__planner_1a81b126a26dcba30ecf7ac7017d54e868`

- :ref:`exhale_typedef_namespacehateb__local__planner_1abacc967cd1313e86a1482c5a70cee691`

- :ref:`exhale_typedef_namespacehateb__local__planner_1a5b4a543316468d9fdac10de1d63e7a77`

- :ref:`exhale_typedef_namespacehateb__local__planner_1aa1464081de429166488fe52d38f977c6`

