OptimizationCostArray
=====================

Message Definition
-----------------

.. code-block:: none

    std_msgs/Header header
    hateb_local_planner/OptimizationCost[] costs

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``costs`` (hateb_local_planner/OptimizationCost[])
    Array of optimization cost entries.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`OptimizationCost Message <OptimizationCost>`

Example Usage
------------

.. code-block:: python

    # Python
    from hateb_local_planner.msg import OptimizationCostArray, OptimizationCost

    a = OptimizationCostArray()
    # a.header.frame_id = "map"
    a.costs.append(OptimizationCost())
