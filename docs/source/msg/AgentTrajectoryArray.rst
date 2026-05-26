AgentTrajectoryArray
===================

Message Definition
-----------------

.. code-block:: none

    AgentTrajectory[]    trajectories

Field Descriptions
-----------------

* ``trajectories`` (cohan_msgs/AgentTrajectory[])
    Array of trajectories for multiple agents.

Links
-----

* :doc:`AgentTrajectory Message <AgentTrajectory>`

Example Usage

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentTrajectoryArray, AgentTrajectory

    # Create an AgentTrajectoryArray message
    array_msg = AgentTrajectoryArray()
    traj = AgentTrajectory()
    array_msg.trajectories.append(traj)

    # Fill trajectories as needed
    array_msg.trajectories[0].trajectory.points.append(...)
