
from core.lib.vehicles.utils import constants as v_c


class LongitudinalControl:
    """Controls the longitudinal movement of the vehicle by controlling gas and brake.

    Attributes:
        driver (object): Driver object specifies safety, aggression parameters of the driver
    """
    def __init__(self, driver):
        self.driver = driver
        return

    def control(self, len_visible_cars, speed, speed_limit, distance_from_front_car, distance_from_end_node,
                traffic_signal_state, steering_angle=0, gas=0, brake=0):
        """
        Chooses which controller to use based on the inputs. Checks if there are any vehicles ahead and determines if cruise control should be used or some other control.

        Args:
            len_visible_cars (int): # of vehicles in front within a certain distance
            speed (float): speed of the vehicle
            speed_limit (float): speed_limit of the road
            distance_from_front_car (float):
            distance_from_end_node (float):
            traffic_signal_state (tuple):
            steering_angle (float): angle in radians
            gas (float): acceleration to give
            brake (float): brake to give

        Returns:
            tuple: (steering_angle, gas, brake)

        """
        if len_visible_cars > 0:    # if there are vehicles ahead, use cruise control
            steering_angle, gas, brake = self.cruise(
                speed, speed_limit,
                distance_from_front_car, steering_angle, gas, brake)
        else:   # if there aren't vehicles, follow if there is a traffic signal
            steering_angle, gas, brake = self.follow_traffic_signals(
                speed=speed, speed_limit=speed_limit,
                distance_from_end_node=distance_from_end_node,
                distance_from_front_car=distance_from_front_car,
                traffic_signal_state=traffic_signal_state)

        return steering_angle, gas, brake

    def cruise(self, speed, speed_limit, distance_from_front_car, steering_angle, gas, brake):
        """
        Provides a basic cruise control assist. Maintains a safe distance from front vehicle, and if there is no front vehicle, follows a safe velocity as determined by the road limit and driver aggression/safety.

        Args:
            speed (float): speed of the vehicle
            speed_limit (float): as determined by the road, traffic rules
            distance_from_front_car (float): distance from front vehicle
            steering_angle (float): angle in radians
            gas (float): acceleration
            brake (float): brake

        Returns:
            tuple: (steering_angle, gas, brake)

        """
        safety = self.driver.safety
        aggression = self.driver.aggression
        # todo: adjust how self.driver affects safe_dist and safe_vel
        safe_dist = max(v_c.CAR_SAFE_DIST*(1 + safety/2), v_c.CAR_SAFE_DIST)

        safe_vel = min(v_c.MAX_SAFE_VEL, speed_limit)
        # safe_vel = max(safe_vel, safe_vel*(1+aggression/2))

        if distance_from_front_car < safe_dist:     # if there is a vehicle within the safe distance to maintain, brake
            brake = 3*(100*speed/(distance_from_front_car+1))
            gas = 0

        else:

            if speed <= safe_vel:
                gas = 0.5*(distance_from_front_car/(speed+1))*abs(safe_vel - speed)
                # todo: normalize gas and brake values and write controllers in temrs of constants, gains
                gas = gas if gas<5 and (speed+gas*0.1<safe_vel) else 5
                brake = 0

            else:
                gas = 0
                brake = 3*(2*speed/distance_from_front_car)

        return steering_angle, gas, brake

    def follow_traffic_signals(self, speed, speed_limit, distance_from_end_node, distance_from_front_car,
                               traffic_signal_state):
        """
        Determines what to do based on the traffic signal state.

        Args:
            speed (float): speed of the vehicle
            speed_limit (float): as determined by traffic rules
            distance_from_end_node (float): distance from the end of the road
            distance_from_front_car (float): distance from the vehicle in front
            traffic_signal_state (tuple): (color, time remaining)

        Returns:
            tuple: (steering_angle, gas, brake)

        """
        steering_angle = 0
        gas = 0
        brake = 0

        color, time_remaining = traffic_signal_state
        if color is not None:

            color, time_remaining = traffic_signal_state

            if color == 'g':

                steering_angle, gas, brake = self.cruise(speed, speed_limit,
                                                         distance_from_front_car=distance_from_front_car,
                                                         steering_angle=steering_angle, gas=gas, brake=brake)

            elif color == 'a':

                steering_angle, gas, brake = self.cruise(speed, speed_limit,
                                                         distance_from_front_car=distance_from_front_car,
                                                         steering_angle=steering_angle, gas=gas, brake=brake)

            elif color == 'r':
                # fixme: when current_speed is close to zero and vehicle far away from traffic light, it should acc
                current_speed = speed
                final_speed = 0
                # todo: calc distance using constants
                distance = distance_from_end_node - 12
                if distance > 0 and current_speed == 0:
                    brake = 0
                    gas = 3
                if distance > 0:
                    brake = current_speed**2/(2*distance)
                    # brake = abs(final_speed**2 - current_speed**2)/50*distance
                else:
                    brake = 10

            else:
                steering_angle, gas, brake = self.cruise(speed, speed_limit,
                                                         distance_from_front_car=distance_from_front_car,
                                                         steering_angle=steering_angle, gas=gas, brake=brake)

        else:   # if there is no traffic signal, follow cruise control to maintain speed limit
            steering_angle, gas, brake = self.cruise(speed, speed_limit,
                                                     distance_from_front_car=distance_from_front_car,
                                                     steering_angle=steering_angle, gas=gas, brake=brake)

        return steering_angle, gas, brake

    def right_of_way(self, intersection_traffic, len_visible_cars, speed, speed_limit, distance_from_front_car,
                     distance_from_end_node, next_turn_direction, steering_angle=0, gas=0, brake=0):
        """
        Encodes the right of way rules at an intersection. For example, a vehicle on a straight road gets preference and goes without stopping, while a vehicle that is turning onto the straight road has to wait.

        Args:
            intersection_traffic (dict): with keys as 1,3,5,7 and values as list of vehicles
            len_visible_cars (int): # of vehicles in front within a certain distance
            speed (float): speed of the vehicle
            speed_limit (float): as determined by traffic rules
            distance_from_front_car (float): distance from the vehicle in front
            distance_from_end_node (float): distance from the end of the road
            next_turn_direction (str): 'right' or 'left' or 'straight'
            steering_angle (float): angle in radians
            gas (float): acceleration to give
            brake (float): brake to give

        Returns:
            tuple: (steering_angle, gas, brake)

        """
        # current zone is always one. Get other traffic at the intersection relative to our zone.
        if len_visible_cars > 0:
            steering_angle, gas, brake = self.cruise(
                speed, speed_limit,
                distance_from_front_car, steering_angle, gas, brake)
        elif distance_from_end_node > 20:
            steering_angle, gas, brake = self.cruise(
                speed, speed_limit,
                distance_from_front_car, steering_angle, gas, brake)

        else:
            if next_turn_direction == "straight":
                # case 1: check cars on 7 (right)

                if len(intersection_traffic[7]) > 0:
                    brake = 1
                    for car in intersection_traffic[7]:
                        if car.speed == 0:
                            continue
                        brake=4
                    if brake==1:
                        brake=0
                        gas=1

                else:
                    steering_angle, gas, brake = self.cruise(
                        speed, speed_limit,
                        distance_from_front_car, steering_angle, gas, brake)

            elif next_turn_direction == "right":
                # case 3: check cars on 3 (left)
                if len(intersection_traffic[3]) > 0:
                    brake = 1
                    for car in intersection_traffic[3]:
                        if car.speed == 0:
                            continue
                        brake = 4
                    if brake == 1:
                        brake = 0
                        gas = 1

                else:
                    steering_angle, gas, brake = self.cruise(
                        speed, speed_limit,
                        distance_from_front_car, steering_angle, gas, brake)

            elif next_turn_direction == "left":
                # case 2: check cars on 7 (right), 5 (straight)
                if (len(intersection_traffic[3]) > 0) or (len(intersection_traffic[5]) > 0) or\
                        (len(intersection_traffic[7]) > 0):
                    brake = 1

                    for car in intersection_traffic[7] + intersection_traffic[3] + intersection_traffic[5]:
                        if car.speed == 0:
                            continue
                        brake = 4
                    if brake == 1:
                        brake = 0
                        gas = 1

                else:
                    steering_angle, gas, brake = self.cruise(
                        speed, speed_limit,
                        distance_from_front_car, steering_angle, gas, brake)

        return steering_angle, gas, brake
