AgentType
=========

Message Definition
-----------------

.. code-block:: none

    # agent types
    int8 ROBOT=0
    int8 HUMAN=1

Constants
--------

* ``ROBOT`` (int8 = 0)
    Represents a robotic agent.

* ``HUMAN`` (int8 = 1)
    Represents a human agent.

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import AgentType

    # Using the constants
    agent_type = AgentType.ROBOT
    agent_type = AgentType.HUMAN

    # Checking agent types
    if agent_type == AgentType.HUMAN:
        print("This is a human agent")
