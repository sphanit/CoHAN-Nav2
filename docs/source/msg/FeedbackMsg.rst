FeedbackMsg
===========

Message Definition
-----------------

.. code-block:: none

    # Message that contains intermediate results 
    # and diagnostics of the (predictive) planner.

    std_msgs/Header header

    # The planned trajectory (or if multiple plans exist, all of them)
    cohan_msgs/Trajectory[] trajectories

    # Index of the trajectory in 'trajectories' that is selected currently
    uint16 selected_trajectory_idx

    # List of active obstacles
    costmap_converter_msgs/ObstacleArrayMsg obstacles_msg

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``trajectories`` (cohan_msgs/Trajectory[])
    Planned trajectories produced by the planner.

* ``selected_trajectory_idx`` (uint16)
    Index of currently selected trajectory.

* ``obstacles_msg`` (costmap_converter_msgs/ObstacleArrayMsg)
    Obstacle array message with detected obstacles.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`Trajectory Message <Trajectory>`
* `costmap_converter_msgs/ObstacleArrayMsg <https://docs.ros.org/en/rolling/api/costmap_converter_msgs/html/msg/ObstacleArrayMsg.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from hateb_local_planner.msg import FeedbackMsg

    fb = FeedbackMsg()
    # fb.header.frame_id = "map"
    # fb.trajectories.append(...)  # use cohan_msgs/Trajectory
