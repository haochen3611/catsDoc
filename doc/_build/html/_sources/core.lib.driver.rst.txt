driver package
=========================

Overview
--------
This module determines the behavior of the driver of the vehicle. Currently implemented attributes are aggression and safety. Aggression is used to draw a sample from a skew distribution and determines the probability with which a driver can break traffic rules (such as breaking the speed limit) or overtaking. Safety is  used to draw a sample from a skew distribution and affects the safety parameters of the vehicle, such as the minimum distance to maintain from the front vehicle (in cruise control).

This module enables simulating a real world like urban traffic scenario by giving different safety and aggression parameters to different drivers. Studies like the effect of selfish drivers (who overtake often) or rash drivers on the traffic evolution can be conducted by using this module. To enable a more detailed study of effect of behavior, the attributes of this module can be extended based on the requirement.


driver\.driver\_attributes module
--------------------------------------------

.. automodule:: core.lib.driver.driver_attributes
    :members:
