TrackedSegment
==============

Message Definition
-----------------

.. code-block:: none

    int8                           type
    geometry_msgs/PoseWithCovariance    pose
    geometry_msgs/TwistWithCovariance   twist
    geometry_msgs/AccelWithCovariance   accel

Field Descriptions
-----------------

* ``type`` (int8)
    Type of the tracked segment. See :doc:`TrackedSegmentType` for possible values.

* ``pose`` (geometry_msgs/PoseWithCovariance)
    Estimated pose of the segment with uncertainty information.

* ``twist`` (geometry_msgs/TwistWithCovariance)
    Estimated velocity of the segment with uncertainty information.

* ``accel`` (geometry_msgs/AccelWithCovariance)
    Estimated acceleration of the segment with uncertainty information.

Links
-----

* `geometry_msgs/PoseWithCovariance Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovariance.html>`_
* `geometry_msgs/TwistWithCovariance Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovariance.html>`_
* `geometry_msgs/AccelWithCovariance Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/AccelWithCovariance.html>`_
* :doc:`TrackedSegmentType Message <TrackedSegmentType>`

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import TrackedSegment

    # Create a TrackedSegment message
    segment = TrackedSegment()

    # Set segment type
    segment.type = 0  # Set appropriate type from TrackedSegmentType

    # Set pose/twist/accel with covariance as needed (geometry_msgs types not imported here)
    segment.pose.pose.position.x = 0.0
    segment.twist.twist.linear.x = 0.0
    segment.accel.accel.linear.x = 0.0
