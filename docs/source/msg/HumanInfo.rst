HumanInfo
=========

Message Definition
-----------------

.. code-block:: none

    int32                 id
    int8                  state
    float32               dist
    string                name
    geometry_msgs/Pose2D  pose
                   
Field Descriptions
-----------------

* ``id`` (int32)
    Unique identifier for the human.

* ``state`` (int8)
    State code for the human (application-specific).

* ``dist`` (float32)
    Distance value associated with the human.

* ``name`` (string)
    Optional human-readable name.

* ``pose`` (geometry_msgs/Pose2D)
    2D pose of the human.

Links
-----

* `geometry_msgs/Pose2D Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose2D.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from agent_path_prediction.msg import HumanInfo

    h = HumanInfo()
    h.id = 1
    h.state = 0
    h.dist = 1.5
    h.name = "human_1"
    # h.pose.x = 0.0
