infrastructure package
=================================

Overview
--------
Infrastructure package is used to model traffic signals, stop lights, pedestrian crossing boards. Currently, the infrastructure implemented are TrafficController and Traffic Signal. TrafficController is  an attribute of a Intersection. It coordinates the switching of signals on the traffic signals on the corresponding intersection. TrafficSignal is an attribute on a Node which is controlled by a TrafficController of the Intersection which has the Node. Each Lane of the Road ending at the Node corresponds to a TrafficSignal on that Node.


infrastructure\.traffic module
-----------------------------------------

.. automodule:: core.lib.infrastructure.traffic
    :members:

    