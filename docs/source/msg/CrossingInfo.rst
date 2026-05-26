CrossingInfo
============

Message Definition
-----------------

.. code-block:: none

    int32[]     agent_ids
    int32[]     indices
    float32[]   times
    float32[]   distances

Field Descriptions
-----------------

* ``agent_ids`` (int32[])
    IDs of the agents involved in the crossing events.

* ``indices`` (int32[])
    Indices referring to trajectory points or agent lists corresponding to the events.

* ``times`` (float32[])
    Times at which crossing events occur.

* ``distances`` (float32[])
    Distances associated with each crossing event.

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import CrossingInfo

    info = CrossingInfo()
    info.agent_ids = [1, 2]
    info.indices = [10, 12]
    info.times = [1.2, 1.3]
    info.distances = [0.5, 0.6]
