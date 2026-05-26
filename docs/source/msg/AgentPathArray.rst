AgentPathArray
==============

Message Definition
-----------------

.. code-block:: none

    Header               header
    AgentPath[]         paths

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``paths`` (cohan_msgs/AgentPath[])
    Array of paths for multiple agents.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`AgentPath Message <AgentPath>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentPathArray, AgentPath

    # Create an AgentPathArray message
    path_array = AgentPathArray()

    # Set header fields as needed (std_msgs/Header not imported here)
    # path_array.header.stamp = ...
    path_array.header.frame_id = "map"

    agent_path = AgentPath()
    path_array.paths.append(agent_path)
