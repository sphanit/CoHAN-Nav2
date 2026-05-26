AgentTrajectoryArrayStamped
===========================

Message Definition
-----------------

.. code-block:: none

    std_msgs/Header              header
    cohan_msgs/AgentTrajectory[] trajectories

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``trajectories`` (cohan_msgs/AgentTrajectory[])
    Array of `AgentTrajectory` messages.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`AgentTrajectory Message <AgentTrajectory>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentTrajectoryArrayStamped, AgentTrajectory

    # Create a message
    msg = AgentTrajectoryArrayStamped()
    # msg.header.frame_id = "map"
    traj = AgentTrajectory()
    msg.trajectories.append(traj)
