AgentPose
=========

Message Definition
-----------------

.. code-block:: none

    int8                       type 
    int32                      id
    geometry_msgs/PoseStamped  pose

Field Descriptions
-----------------

* ``type`` (int8)
    Type identifier for the agent pose.

* ``id`` (int32)
    Unique identifier for the agent.

* ``pose`` (geometry_msgs/PoseStamped)
    Pose with timestamp for the agent.

Links
-----

* `geometry_msgs/PoseStamped Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from agent_path_prediction.msg import AgentPose

    ap = AgentPose()
    ap.type = 0
    ap.id = 1
    # ap.pose.header.frame_id = "map"
    # ap.pose.pose.position.x = 0.0
