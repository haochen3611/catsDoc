import math
from core.lib.maps.utils import constants as road_c
from core.lib.physics.utils import constants as sim_c


def two_wheel_drive(x, y, heading, speed, length, steering_angle, gas, brake, gas_to_acc=1, brake_to_acc=1):
    """Simulates the physics of a front two wheel drive model.

    Args:
        x (float): x coordinate of car
        y (float): y coordinate of car
        heading (float): in radians
        speed (float): in meters/sec
        length (float): in meters
        steering_angle (float): in radians
        gas (float): acceleration to give. Normalized in 0 to 1
        brake (float): brake to give. Normalized in 0 to 1
        gas_to_acc (float): constant specifying the multiplying factor
        brake_to_acc (float): constant specifying the multiplying factor

    Returns:
        tuple: (x, y, heading, speed)
    """

    front_wheel_x = x + length / 2 * math.cos(heading)
    front_wheel_y = y + length / 2 * math.sin(heading)
    back_wheel_x = x - length / 2 * math.cos(heading)
    back_wheel_y = y - length / 2 * math.sin(heading)

    speed += (
        gas * gas_to_acc * sim_c.DT - (
            brake * brake_to_acc * sim_c.DT) - road_c.DRAG_COEF * speed * sim_c.DT)
    speed = speed if speed >= 0 else 0

    # update wheel positions
    front_wheel_x += speed * sim_c.DT * math.cos(heading + steering_angle)
    front_wheel_y += speed * sim_c.DT * math.sin(heading + steering_angle)
    back_wheel_x += speed * sim_c.DT * math.cos(heading)
    back_wheel_y += speed * sim_c.DT * math.sin(heading)

    # update car position and heading
    x = (front_wheel_x + back_wheel_x) / 2
    y = (front_wheel_y + back_wheel_y) / 2
    heading = math.atan2((front_wheel_y - back_wheel_y), (front_wheel_x - back_wheel_x))

    return x, y, heading, speed


def four_wheel_drive(x, y, heading, speed, length, steering_angle, gas, brake, gas_to_acc=1, brake_to_acc=1):
    """Simulates the physics of all wheel drive model

    Args:
        x (float): x coordinate of car
        y (float): y coordinate of car
        heading (float): in radians
        speed (float): in meters/sec
        length (float): in meters
        steering_angle (float): in radians
        gas (float): acceleration to give. Normalized in 0 to 1
        brake (float): brake to give. Normalized in 0 to 1
        gas_to_acc (float): constant specifying the multiplying factor
        brake_to_acc (float): constant specifying the multiplying factor

    Returns:
        tuple: (x, y, heading, speed)

    Notes:
        Not implemented yet.

    """
    
    return x, y, heading, speed
