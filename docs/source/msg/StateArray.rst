StateArray
==========

Message Definition
-----------------

.. code-block:: none

    int8[] states

Field Descriptions
-----------------

* ``states`` (int8[])
    Array of state values representing the states of multiple entities.

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import StateArray

    # Create a StateArray message
    state_array = StateArray()

    # Add states to the array
    state_array.states = [0, 1, 2]  # Example states
