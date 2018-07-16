adas package
=======================

Overview
--------
ADAS stands for Advanced Driver Assistance System. This module takes in a command from the decision module and accordingly outputs a control signal which comprises of steering angle, gas, and brake.  ADAS module has the two classes Longitudinal Control, which  contains methods to output a longitudinal control (i.e. gas, brake) in different scenarios such as cruise control mode, lane change mode, turning at an intersection, stopping at a red signal etc, and LateralControl, which contains methods to output a lateral control signal (i.e. steering angle) in scenarios such as lane change, turning at an intersection, lane centering etc. 

In either of the above classes, the methods use a very simple PID controller which use a reference velocity or distance (in longitudinal control) and a reference heading direction (in lateral control) for generating a control signal so that the vehicles next state closely matches the reference values used. Since the initial goal was to have a basic frame work to integrate all the modules, no sophisticated control models were developed. As this software is modular, it is fairly straightforward to improve upon the implemented control algorithms.


adas\.driving\_model module
--------------------------------------

.. automodule:: core.lib.adas.driving_model
    :members:
    
adas\.intersection\_model module
-------------------------------------------

.. automodule:: core.lib.adas.intersection_model
    :members:
 
adas\.lane\_center module
------------------------------------

.. automodule:: core.lib.adas.lane_center
    :members:
    
adas\.lane\_change module
------------------------------------

.. automodule:: core.lib.adas.lane_change
    :members:
    
adas\.longitudinal\_contrl module
--------------------------------------------

.. automodule:: core.lib.adas.longitudinal_contrl
    :members:
    
adas\.rules module
-----------------------------

.. automodule:: core.lib.adas.rules
    :members:
    
