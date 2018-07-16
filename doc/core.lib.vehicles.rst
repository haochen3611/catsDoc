vehicles package
===========================

Overview
--------
Vehicles module provides with classes and methods to create vehicles of different types, and a base Vehicle class that contains all the basic functionalities of any vehicle. We adopted a factory design pattern for the creation of a vehicle wherein, the creation of every Vehicle is forced through a common factory class VehicleFactory which has a build_vehicle method, which takes the vehicle config such as vehicle type, sensors, engine and instantiates the corresponding Vehicle object, and place_on_map which takes a start node and end node of type CATS.maps.Node and places it on the map.

We implemented different types of vehicles which inherit the base Vehicle class. As of now, we have Car, Truck, Bus and Intelligent Car. The different types of vehicles differ in their physical attributes like length, width, engine, sensors, decision modules, etc. IntelligentCar is the utopian smart car with complete autonomy, V2X communication, adherence to traffic rules etc. Developing the decision module of this type of vehicle is the main goal of this software.


vehicles\.bus module
-------------------------------

.. automodule:: core.lib.vehicles.bus
    :members:

vehicles\.car module
-------------------------------

.. automodule:: core.lib.vehicles.car
    :members:

vehicles\.truck module
---------------------------------

.. automodule:: core.lib.vehicles.truck
    :members:

vehicles\.utils module
---------------------------------

.. automodule:: core.lib.vehicles.utils
    :members:

vehicles\.vehicle module
-----------------------------------

.. automodule:: core.lib.vehicles.vehicle
    :members:

vehicles\.vehicle\_factory module
--------------------------------------------

.. automodule:: core.lib.vehicles.vehicle_factory
    :members:


Module contents
---------------

.. automodule:: core.lib.vehicles
    :members:
