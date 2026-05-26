AgentPath
=========

Message Definition
-----------------

.. code-block:: none

    Header           header
    uint64          id
    nav_msgs/Path   path

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``id`` (uint64)
    Unique identifier for the agent.

* ``path`` (nav_msgs/Path)
    The planned or tracked path of the agent.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* `nav_msgs/Path Message <http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentPath

    # Create an AgentPath message
    agent_path = AgentPath()

    # Set header fields as needed (std_msgs/Header not imported here)
    # agent_path.header.stamp = ...
    agent_path.header.frame_id = "map"

    agent_path.id = 1

    # Set path (nav_msgs/Path) fields as needed
    agent_path.path.poses.append(...)
