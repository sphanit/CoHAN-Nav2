AgentTimeToGoalArray
===================

Message Definition
-----------------

.. code-block:: none

    Header                  header
    AgentTimeToGoal[]      times_to_goal

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``times_to_goal`` (cohan_msgs/AgentTimeToGoal[])
    Array of estimated times to goal for multiple agents.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`AgentTimeToGoal Message <AgentTimeToGoal>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentTimeToGoalArray, AgentTimeToGoal

    # Create an AgentTimeToGoalArray message
    array_msg = AgentTimeToGoalArray()

    # Set header fields as needed (std_msgs/Header not imported here)
    # array_msg.header.stamp = ...
    array_msg.header.frame_id = "map"

    time_to_goal = AgentTimeToGoal()
    array_msg.times_to_goal.append(time_to_goal)
