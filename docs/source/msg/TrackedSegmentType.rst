TrackedSegmentType
=================

Message Definition
-----------------

.. code-block:: none

    # constants, adapted from human avatar in morse simulator
    int8    HEAD=0
    int8    TORSO=1
    int8    RIGHT_SHOULDER=2
    int8    RIGHT_ELBOW=3
    int8    RIGHT_WRIST=4
    int8    RIGHT_HIP=5
    int8    RIGHT_KNEE=6
    int8    RIGHT_ANKLE=7
    int8    LEFT_SHOULDER=8
    int8    LEFT_ELBOW=9
    int8    LEFT_WRIST=10
    int8    LEFT_HIP=11
    int8    LEFT_KNEE=12
    int8    LEFT_ANKLE=13

Constants
---------

* ``HEAD`` (int8 = 0)
    Head segment of the human avatar.

* ``TORSO`` (int8 = 1)
    Torso segment of the human avatar.

* ``RIGHT_SHOULDER`` (int8 = 2)
    Right shoulder segment.

* ``RIGHT_ELBOW`` (int8 = 3)
    Right elbow segment.

* ``RIGHT_WRIST`` (int8 = 4)
    Right wrist segment.

* ``RIGHT_HIP`` (int8 = 5)
    Right hip segment.

* ``RIGHT_KNEE`` (int8 = 6)
    Right knee segment.

* ``RIGHT_ANKLE`` (int8 = 7)
    Right ankle segment.

* ``LEFT_SHOULDER`` (int8 = 8)
    Left shoulder segment.

* ``LEFT_ELBOW`` (int8 = 9)
    Left elbow segment.

* ``LEFT_WRIST`` (int8 = 10)
    Left wrist segment.

* ``LEFT_HIP`` (int8 = 11)
    Left hip segment.

* ``LEFT_KNEE`` (int8 = 12)
    Left knee segment.

* ``LEFT_ANKLE`` (int8 = 13)
    Left ankle segment.

Example Usage
------------

.. code-block:: python

    # Python
    from cohan_msgs.msg import TrackedSegmentType

    # Using segment type constants
    segment_type = TrackedSegmentType.HEAD
    
    # Checking segment types
    if segment_type == TrackedSegmentType.TORSO:
        print("This is a torso segment")
    elif segment_type == TrackedSegmentType.LEFT_HAND:
        print("This is a left hand segment")
