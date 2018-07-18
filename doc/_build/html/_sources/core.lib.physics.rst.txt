physics package
==========================

Overview
--------
This is the module that is responsible for the movement of the vehicle in the simulation. Each vehicle is initialized with a physics module that determines the characteristics such as engine, fuel consumption, vehicle dynamics etc. Currently a front wheel drive model is implemented under vehicle dynamics, and can be extended later to more complex models that include tire force modeling, longitudinal and lateral forces, etc.

This module takes control signal as the input from the vehicle decision module. The control signal comprises of steering, gas and brake values, which are computed by the ADAS module according to the decision taken. Currently, a simple model is implemented that does not consider all the forces acting on the vehicle, rotation of the tires, etc. Given the control signal, the velocity of the front wheels is modified as *v = u + a t*, where *u* is the velocity in the previous time instant, a is the gas or the brake value (negative), and *v* is the velocity calculated at the current time instant. Using this *v*, the new new coordinates of the front tires are calculated, and the coordinate of the center of the line joining them is calculated. Using this center and the updated vehicle heading (from the steering signal), the updated location of the center of gravity of the vehicle is calculated, and correspondingly of the entire chasis. The updated coordinates of the vehicle (coordinates of the center of gravity, and the heading of the vehicle) is updated in the simulation so that the display can be updated.


physics\.update\_car\_states module
----------------------------------------------

.. automodule:: core.lib.physics.update_car_states
    :members:
