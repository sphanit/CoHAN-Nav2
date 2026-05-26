AgentMarkerStamped
==================

Message Definition
-----------------

.. code-block:: none

    Header             header
    AgentMarker       agent

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``agent`` (cohan_msgs/AgentMarker)
    The agent marker information.

Links
-----

* :doc:`AgentMarker Message <AgentMarker>`
* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentMarkerStamped, AgentMarker

    # Create an AgentMarkerStamped message
    stamped_marker = AgentMarkerStamped()

    # Set header fields as needed (std_msgs/Header not imported here)
    # stamped_marker.header.stamp = ...
    stamped_marker.header.frame_id = "map"

    # Fill nested AgentMarker
    stamped_marker.agent = AgentMarker()
    stamped_marker.agent.id = 1
    stamped_marker.agent.active = True
