PassageType
===========

Message Definition
-----------------

.. code-block:: none

    int8 OPEN=0
    int8 DOOR=1
    int8 PILLAR=2
    int8 WALL=3

    int8 type

Constants
---------

* ``OPEN`` (int8 = 0)
    Represents an open passage with no obstacles.

* ``DOOR`` (int8 = 1)
    Represents a passage through a door.

* ``PILLAR`` (int8 = 2)
    Represents a passage between pillars.

* ``WALL`` (int8 = 3)
    Represents a passage along a wall.

Field Descriptions
-----------------

* ``type`` (int8)
    The type of passage, using one of the defined constants.

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import PassageType

    # Create a PassageType message
    passage = PassageType()
    
    # Set passage type using constants
    passage.type = PassageType.DOOR  # For a door passage
    
    # Check passage type
    if passage.type == PassageType.OPEN:
        print("This is an open passage")
    elif passage.type == PassageType.WALL:
        print("This is a wall passage")
