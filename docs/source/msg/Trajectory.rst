Trajectory
==========

Message Definition
-----------------

.. code-block:: none

    TrajectoryPoint[]    points

Field Descriptions
-----------------

* ``points`` (TrajectoryPoint[])
    Array of trajectory points representing a path through space and time.

Links
-----

* :doc:`TrajectoryPoint Message <TrajectoryPoint>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import Trajectory, TrajectoryPoint

    # Create a Trajectory message
    trajectory = Trajectory()

    # Add trajectory points
    point = TrajectoryPoint()
    # Configure point fields as needed
    trajectory.points.append(point)

    # Add more points to define the complete trajectory
