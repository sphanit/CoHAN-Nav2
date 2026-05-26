AgentTimeToGoal
==============

Message Definition
-----------------

.. code-block:: none

    Header      header
    uint64      id
    int8        type
    duration    time_to_goal

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``id`` (uint64)
    Unique identifier for the agent.

* ``type`` (int8)
    Type identifier for the agent. See :doc:`AgentType` for possible values.

* ``time_to_goal`` (duration)
    Estimated time remaining for the agent to reach its goal.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`AgentType Message <AgentType>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentTimeToGoal

    # Create an AgentTimeToGoal message
    time_to_goal = AgentTimeToGoal()

    # Set header fields as needed (std_msgs/Header not imported here)
    # time_to_goal.header.stamp = ...
    time_to_goal.header.frame_id = "map"

    time_to_goal.id = 1
    time_to_goal.type = 0

    # Set duration
    time_to_goal.time_to_goal.sec = 30
