sensors package
==========================

Overview
--------
Sensors act as an interface between the environment and the vehicles intelligence modules. This module acts as a simulation of real world sensors, by taking in in formation from the vehicle and the maps module about the environment around the vehicle. The currently implemented sensors include BasicVisual, GPS and Speed. BasicVisual provides information about the objects present in the field of view of the vehicle This can be thought of as LIDAR or RADAR readings, or simply as the surroundings as seen by a driver through the windshield or the rear view mirrors. GPS provides the coordinates of the vehicle and Speed provides the speedometer reading of the vehicle. These sensor readings are used by the vehicles internal modules to localize the vehicle and to take a decision based on the inferences. To understand the effect of sensor failures/malfunctions, these sensors can be configured to provide noisy readings to the vehicle.


sensors\.sensor module
---------------------------------

.. automodule:: core.lib.sensors.sensor
    :members:

sensors\.sensor\_factory module
------------------------------------------

.. automodule:: core.lib.sensors.sensor_factory
    :members:

