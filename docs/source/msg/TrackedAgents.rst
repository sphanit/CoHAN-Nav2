TrackedAgents
=============

Message Definition
-----------------

.. code-block:: none

    Header               header
    TrackedAgent[]       agents

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``agents`` (TrackedAgent[])
    Array of tracked agents in the environment.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`TrackedAgent Message <TrackedAgent>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import TrackedAgents, TrackedAgent

    # Create a TrackedAgents message
    tracked_agents = TrackedAgents()

    # Set header fields as needed (std_msgs/Header not imported here)
    # tracked_agents.header.stamp = ...
    tracked_agents.header.frame_id = "map"

    agent = TrackedAgent()
    tracked_agents.agents.append(agent)
