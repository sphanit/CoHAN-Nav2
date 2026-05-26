TrajectoryArray
==============

Message Definition
-----------------

.. code-block:: none

    Header            header
    Trajectory[]      trajectories

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``trajectories`` (Trajectory[])
    Array of trajectories.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`Trajectory Message <Trajectory>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import TrajectoryArray, Trajectory

    # Create a TrajectoryArray message
    traj_array = TrajectoryArray()

    # Set header fields as needed (std_msgs/Header not imported here)
    # traj_array.header.stamp = ...
    traj_array.header.frame_id = "map"

    trajectory = Trajectory()
    traj_array.trajectories.append(trajectory)
