maps package
=======================

Overview
--------
Imagine the diffculty in driving in a new city without the help of a maps service on your phone or car. Even for experienced cab drivers, it is often diffcult to navigate through city traffic, so let us not even discuss the plight of rookie drivers. It is easy to understand that modeling maps is central to the effectiveness of any simulation software. Self-driving start-ups are investing heavily in developing HD maps that explain the fine details of the environment around the vehicle. Putting it simply, the more realistic the maps look, the better the software.

Similarly, modeling maps is central to the effectiveness of CATS. The most important requirements/features of maps are:
    
* Maps should represent an urban traffic scenario, along with the landscape of the city
* Maps require a detailed modeling of roads, intersections, lanes, etc
* Maps should encode the topological connections between roads, for routing
* Specific to CATS, maps also store the spatial details of all the vehicles in the simulation.


maps\.map module
---------------------------

.. automodule:: core.lib.maps.map
    :members:
    