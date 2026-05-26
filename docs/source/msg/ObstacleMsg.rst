ObstacleMsg
===========

Message Definition
-----------------

.. code-block:: none

    # Message that contains a list of polygon shaped obstacles.
    # Special types:
    # Polygon with 1 vertex: Point obstacle
    # Polygon with 2 vertices: Line obstacle
    # Polygon with more than 2 vertices: First and last points are assumed to be connected
    
    std_msgs/Header header

    # Actual obstacle positions (polygon descriptions)
    geometry_msgs/PolygonStamped[] obstacles

    # Obstacle IDs [optional]
    uint32[] ids

    # Individual orientations (centroid) [optional]
    geometry_msgs/QuaternionStamped[] orientations

    # Individual velocities (centroid) [optional]
    geometry_msgs/TwistWithCovariance[] velocities

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS header.

* ``obstacles`` (geometry_msgs/PolygonStamped[])
    List of polygon obstacles.

* ``ids`` (uint32[])
    Optional obstacle IDs.

* ``orientations`` (geometry_msgs/QuaternionStamped[])
    Optional orientations for obstacle centroids.

* ``velocities`` (geometry_msgs/TwistWithCovariance[])
    Optional centroid velocities.

Links
-----

* `geometry_msgs/PolygonStamped Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/PolygonStamped.html>`_
* `geometry_msgs/QuaternionStamped Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/QuaternionStamped.html>`_
* `geometry_msgs/TwistWithCovariance Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovariance.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from hateb_local_planner.msg import ObstacleMsg

    om = ObstacleMsg()
    # om.header.frame_id = "map"
    # om.obstacles.append(...)  # use geometry_msgs/PolygonStamped
