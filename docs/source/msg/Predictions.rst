Predictions
===========

Message Definition
-----------------

.. code-block:: none

    std_msgs/Float32MultiArray predictions
    std_msgs/Float32MultiArray logits
    std_msgs/Int32 num_agents
    std_msgs/Int32 history_length
    std_msgs/Int32 window_length
    std_msgs/Int32 max_agents
    std_msgs/Float32 timestep

Field Descriptions
-----------------

* ``predictions`` (std_msgs/Float32MultiArray)
    Array containing the predicted trajectories for agents.

* ``logits`` (std_msgs/Float32MultiArray)
    Array containing the logit values for predictions.

* ``num_agents`` (std_msgs/Int32)
    Current number of agents being tracked.

* ``history_length`` (std_msgs/Int32)
    Length of the historical data used for predictions.

* ``window_length`` (std_msgs/Int32)
    Length of the prediction window.

* ``max_agents`` (std_msgs/Int32)
    Maximum number of agents that can be tracked.

* ``timestep`` (std_msgs/Float32)
    Time step between predictions.

Links
-----

* `std_msgs/Float32MultiArray Message <http://docs.ros.org/en/api/std_msgs/html/msg/Float32MultiArray.html>`_
* `std_msgs/Int32 Message <http://docs.ros.org/en/api/std_msgs/html/msg/Int32.html>`_
* `std_msgs/Float32 Message <http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import Predictions

    # Create a Predictions message
    pred = Predictions()
    # For std_msgs wrapper fields set .data as needed
    pred.num_agents.data = 3
    pred.history_length.data = 20
    pred.window_length.data = 50
    pred.max_agents.data = 10
    pred.timestep.data = 0.1

    # Initialize arrays as needed
    pred.predictions = Float32MultiArray()
    pred.logits = Float32MultiArray()
