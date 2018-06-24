from core.lib.maps.utils import constants as road_c
from core.lib.physics.utils import constants as car_c
import math
# ToDo: Add lane center capability


class LaneCenter:
    """ Contains two methods that keep a autonomous vehicle driving in the center of the lane.

    """
    def __init__(self):
        return

    def control_steering(self, sensor_reading, lane_change_flag, lane_center_dy_flag, heading_control_flag,
                         road_width, road_heading):

        """
        Controller to steer the vehicle given the input observations. Abstracts out a reference heading to follow using
        the given inputs and calls `heading_control` to get a steering output.

        Notes:
            The several flags used here determine which controller is acting on the vehicle at that instant. Only one
            controller acts at a time, and selects a reference heading which is maintained using `heading_control`.

        Args:
            sensor_reading (tuple): (dy, dtheta) readings
            lane_change_flag (bool): True if the vehicle is changing lane
            lane_center_dy_flag (bool): True if vehicle is trying to control the distance from the lane center
            heading_control_flag (bool): True if vehicle is trying to control heading wrt road heading
            road_width (float): width of road
            road_heading (float): heading direction of the road in radians

        Returns:
            tuple: (steering_angle, lane_change_flag, lane_center_dy_flag, heading_control_flag)
        """
        lane_center_dy, lane_center_dtheta = sensor_reading

        # normalized_lane_center_dy = float(lane_center_dy) / road_width

        # const = -1

        car_heading = road_heading - lane_center_dtheta

        if lane_change_flag:    # if the car is in the process of changing a lane
            if lane_center_dy < 0:
                reference_heading = road_heading + math.pi/8
            else:
                reference_heading = road_heading - math.pi/8

            steering_angle = self.heading_control(car_heading, reference_heading)

            if abs(lane_center_dy) < road_width/8:  # turn off the lane change flag when it is close to the center
                lane_change_flag = False
        else:
            reference_heading = road_heading    # if it is not changing lane, give the reference heading as road heading

            if not heading_control_flag:

                if abs(reference_heading - car_heading) > math.pi/120 and not lane_center_dy_flag:
                    heading_control_flag = True
                    steering_angle = self.heading_control(car_heading, reference_heading)
                elif abs(reference_heading - car_heading) > math.pi/120 and lane_center_dy_flag:
                    if abs(lane_center_dy) > road_width/16:
                        lane_center_dy_flag = True
                        reference_heading = road_heading - lane_center_dy/abs(lane_center_dy)* math.pi/20
                        steering_angle = self.heading_control(car_heading, reference_heading)

                    else:
                        lane_center_dy_flag = False
                        steering_angle = self.heading_control(car_heading, reference_heading)

                elif abs(lane_center_dy) > road_width/16:
                    lane_center_dy_flag = True
                    reference_heading = road_heading - lane_center_dy/abs(lane_center_dy)* math.pi/20
                    steering_angle = self.heading_control(car_heading, reference_heading)

                else:
                    heading_control_flag = False
                    steering_angle = self.heading_control(car_heading, reference_heading)

            else:
                if abs(reference_heading - car_heading) > math.pi/120:
                    heading_control_flag = True
                    steering_angle = self.heading_control(car_heading, reference_heading)
                else:
                    heading_control_flag = False
                    steering_angle = self.heading_control(car_heading, reference_heading)

        return steering_angle, lane_change_flag, lane_center_dy_flag, heading_control_flag

    def heading_control(self, car_heading, reference_heading):
        """
        Controller to provide a steering to align car_heading with a given reference_heading

        Args:
            car_heading (float): angle in radians
            reference_heading (float): angle in radians

        Returns:
            float: steering_angle in radians

        """

        diff = reference_heading - car_heading
        if abs(diff) > math.pi:
            diff = 2*math.pi - abs(diff)

        theta_const = 0.8
        if abs(diff) < car_c.MAX_STEERING_ANGLE:
            steering_angle = diff * theta_const
        else:
            steering_angle = car_c.MAX_STEERING_ANGLE * diff / abs(diff)

        return steering_angle


