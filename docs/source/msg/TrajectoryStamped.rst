TrajectoryStamped
================

Message Definition
-----------------

.. code-block:: none

    Header              header
    TrajectoryPoint[]   points

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``points`` (TrajectoryPoint[])
    Array of trajectory points with timing information.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`TrajectoryPoint Message <TrajectoryPoint>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import TrajectoryStamped, TrajectoryPoint

    # Create a TrajectoryStamped message
    traj = TrajectoryStamped()

    # Set header fields as needed (std_msgs/Header not imported here)
    # traj.header.stamp = ...
    traj.header.frame_id = "map"

    point = TrajectoryPoint()
    traj.points.append(point)
