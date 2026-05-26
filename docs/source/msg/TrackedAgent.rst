TrackedAgent
============

Message Definition
-----------------

.. code-block:: none

    # states
    int8 STATIC=0
    int8 MOVING=1
    int8 STOPPED=2
    int8 BLOCKED=3

    uint64            track_id
    int8              state
    int8              type
    string            name
    TrackedSegment[]  segments

Constants
---------

* ``STATIC`` (int8 = 0)
    Agent is stationary and not expected to move.

* ``MOVING`` (int8 = 1)
    Agent is currently in motion.

* ``STOPPED`` (int8 = 2)
    Agent has temporarily stopped but may move.

* ``BLOCKED`` (int8 = 3)
    Agent's motion is blocked by obstacles.

Field Descriptions
-----------------

* ``track_id`` (uint64)
    Unique identifier for tracking the agent.

* ``state`` (int8)
    Current state of the agent (STATIC, MOVING, STOPPED, or BLOCKED).

* ``type`` (int8)
    Type of the agent. See :doc:`AgentType` for possible values.

* ``name`` (string)
    Human-readable name or identifier for the agent.

* ``segments`` (TrackedSegment[])
    Array of tracked segments that make up the agent.

Links
-----

* :doc:`TrackedSegment Message <TrackedSegment>`
* :doc:`AgentType Message <AgentType>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import TrackedAgent, TrackedSegment

    # Create a TrackedAgent message
    agent = TrackedAgent()
    
    # Set basic properties
    agent.track_id = 1
    agent.state = TrackedAgent.MOVING
    agent.type = 1  # Set appropriate type from AgentType
    agent.name = "human_1"
    
    # Add tracked segments
    segment = TrackedSegment()
    # Configure segment...
    agent.segments.append(segment)
