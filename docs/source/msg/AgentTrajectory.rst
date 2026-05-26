AgentTrajectory
==============

Message Definition
-----------------

.. code-block:: none

    uint64              id
    int8                type
    Trajectory          trajectory

Field Descriptions
-----------------

* ``id`` (uint64)
    Unique identifier for the agent.

* ``type`` (int8)
    Type identifier for the agent. See :doc:`AgentType` for possible values.

* ``trajectory`` (cohan_msgs/Trajectory)
    The planned or predicted trajectory of the agent.

Links
-----

* :doc:`Trajectory Message <Trajectory>`
* :doc:`AgentType Message <AgentType>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentTrajectory, Trajectory

    # Create an AgentTrajectory message
    agent_traj = AgentTrajectory()
    agent_traj.id = 1
    agent_traj.type = 0
    agent_traj.trajectory = Trajectory()

    # Fill trajectory fields as needed
    agent_traj.trajectory.points.append(...)
