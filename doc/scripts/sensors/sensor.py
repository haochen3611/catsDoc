from ctypes import *
import ctypes
import math
from core.helpers import perpendicular_distance_of_point_from_line
from shapely.geometry import Polygon, LineString, Point
from core.lib.maps.utils import constants as road_c
from core.lib.vehicles.utils import constants as v_c

# future: Currently passing car objects to sensors. In future, only pass the required attributes of car via an interface


class GPS:
    """Simulates a GPS device.

    """
    @staticmethod
    def get_position(x, y):
        """Shows the GPS coordinates of vehicle.

        Args:
            x (float): x coordinate of vehicle
            y (float): y coordinate of vehicle

        Returns:
            list: [x, y] of the vehicle

        """
        return GPS.__get_noisy(x, y)

    # todo: implement noisy version of gps
    @staticmethod
    def __get_noisy(x, y):
        """
        Implement noisy version of the gps, with normal distribution or some skewed distribution.

        Args:
            x (float): x coordinate of vehicle
            y (float): y coordinate of vehicle

        Returns:
            list: [x, y] with added noise

        Notes:
            Still under development.

        """
        return [x, y]


class VehicleSpeedSensor:
    """Simulates a speed sensor.

    Attributes:
        car (object): Vehicle object

    Todo: Implement noisy version of speed.
    
    Future: Implement sensor fusion using GPS data.

    """
    def __init__(self, car=None):
        self.car = car

    def read_speed(self):
        """Shows the speed of vehicle.

        Returns:
            float: speed

        """
        return self.__get_noisy()

    # 
    def __get_noisy(self):
        """
        Implement a noisy version of the speed. Also implement sensor fusion by taking data from GPS sensors.

        Returns:
            float: speed

        Notes:
            Still under development.

        """
        return self.car.speed



class LaneCenterSensor:
    """Simulates a sensor that gives offset from the lane center.


    Future: Implementation might have to change for curved roads/roadstring ds.

    """
    def __init__(self):
        pass

    @staticmethod
    def get_lane_center_dtheta(car):
        """
        Takes in the car object and returns the difference between the heading of the road/lane and the heading of the vehicle.

        Args:
            car (object): vehicle object

        Returns:
            float: angle in radians

        """
        lane_center_dtheta = car.get_current_road().heading[car._current_segment] - car.get_heading()
        if lane_center_dtheta > math.pi:
            # edge case when road heading is pi and car heading is -pi.
            # math.atan2 range is (-pi, pi]. So -pi is not included.
            lane_center_dtheta -= 2*math.pi
        return lane_center_dtheta

    @staticmethod
    def get_lane_center_dy(car):
        """
        Takes in car object and returns the displacement from the lane center.

        Args:
            car (object): vehicle object

        Returns:
            float: distance from center of lane

        Notes:
            Calculates distance from left edge of road and subtracts lane number * lane width.

        """

        current_road = car.get_current_road()

        left_line_string = current_road.left_line_string
        point = Point(car.x, car.y)
        llsx, llsy = left_line_string.xy
        lx, ly = current_road.line_string.xy

        slope = math.atan2(llsy[1]-llsy[0], llsx[1]-llsx[0])
        c = llsy[1] - math.tan(slope)*llsx[1]
        if abs(c)>1e3:
            left_edge_dy = abs(llsx[1] - car.x)
        else:
            left_edge_dy = abs(car.y - math.tan(slope)*car.x - c)/math.sqrt(math.tan(slope)**2 + 1)

        lane_dy = left_edge_dy - (current_road.lane_width*(1 + car.get_lane_no()) - current_road.lane_width/2)

        # lane_dy = self.offset_direction(car) * abs(current_road.get_distance_from_road_center(car.x, car.y) - \
        #                                abs(car.get_lane_no()-current_road.no_of_lanes//2) * current_road.lane_width)

        return lane_dy

    @staticmethod
    def offset_direction(car):
        """Deprecated. When lane center was calculated using distance from center of road, this method was used to check if the car is to the left of the road center or to the right.

        Args:
            car (object): car

        Returns:
            int: 0 or 1 or -1

        Notes:
            Not using this currently. 

        """
        current_lane = car.get_current_road().lanes[car.get_lane_no()]
        _,_,end_x, end_y = current_lane.lane_center_xy
        road_theta = car.get_current_road().heading[car._current_segment]
        car_theta = math.atan2((end_y - car.y), (end_x - car.x))

        if road_theta == math.pi:
            if math.pi - car_theta < road_theta:
                sign = 1
                # lane = self.no_of_lanes // 2 + ind
            elif math.pi - car_theta > road_theta:
                sign = -1
                # lane = self.no_of_lanes // 2 - ind
            else:
                sign = 0
        else:
            if car_theta < road_theta:
                sign = 1
                # lane = self.no_of_lanes // 2 + ind
            elif car_theta > road_theta:
                sign = -1
                # lane = self.no_of_lanes // 2 - ind
            else:
                sign = 0

        return sign


class PositionBasedVelocityEstimator:
    """
    Estimates the vehicle speed based on its position.

    Notes:
        Not implemented currently. returning the speed directly.

    Future: implement a fusion of speed sensor and position based velocity estimator.

    """
    def __init__(self):
        pass

    @staticmethod
    def calc_speed(car):
        if len(car.x_history)<=1:
            return car.speed
        velocity = [
        ]
        return math.sqrt(velocity[0]**2+velocity[1]**2)

    @staticmethod
    def read_speed(speed):
        return speed


class BasicVisual:
    """
    Simulates the capability of a human visual system.

    """

    def __init__(self):
        pass

    @staticmethod
    def get_visible_cars(car, tg):
        """
        Shows cars that are visible to a human driver. Similar to what a driver can see (directly or via the mirrors, side or rear).

        Args:
            car (object): Vehicle object
            tg (object): Traffic Graph object

        Returns:
            dict: with keys as ['front', 'back', 'left', 'right'] and values as a list of vehicle objects

        Future: instead of passing a car, pass only the attributes as a message or via a bus

        """

        
        lane_no = car.get_lane_no()

        current_road = car.get_current_road()
        lane_width = current_road.lane_width

        visible_cars = dict.fromkeys(['front', 'back', 'left', 'right'], [])

        fbox = get_car_bounding_box_front(car)

        visible_cars['front'] = tg.get_vehicles_by_bounding_box(bounding_box=fbox.bounds, objects=False)

        if lane_no is None:
            print("Error: lane_no is None")
        else:
            if lane_no == 0:
                rbox = get_car_bounding_box_adj(car, lane_width, direction="right")
                visible_cars['right'] = tg.get_vehicles_by_bounding_box(bounding_box=rbox.bounds, objects=False)

            elif lane_no == current_road.no_of_lanes - 1:
                lbox = get_car_bounding_box_adj(car, lane_width, direction="left")
                visible_cars['left'] = tg.get_vehicles_by_bounding_box(bounding_box=lbox.bounds, objects=False)
            else:
                rbox = get_car_bounding_box_adj(car, lane_width, direction="right")
                visible_cars['right'] = tg.get_vehicles_by_bounding_box(bounding_box=rbox.bounds, objects=False)

                lbox = get_car_bounding_box_adj(car, lane_width, direction="left")
                visible_cars['left'] = tg.get_vehicles_by_bounding_box(bounding_box=lbox.bounds, objects=False)

        #   rtree intersection method only outputs object ids found in axis aligned boxes. So we need to filter out the
        #   vehicles that are not in the actual polygon required. We use shapely polygon.contains()
        for key in visible_cars.keys():
            temp = []
            for v in visible_cars[key]:
                if not (car == tg.get_vehicle(v)):    # checks if the returned object is not itself
                    if tg.get_vehicle(v).is_locked():
                        #   checks if the vehicle is locked. Refer to vehicle class for what locked means
                        pass
                    else:
                        # tv = tg.get_vehicle(v)
                        # temp.append(tv)
                        if key=="right":
                            tv = tg.get_vehicle(v)
                            if rbox.contains(Point(tv.x, tv.y)):
                                temp.append(tv)
                        elif key=="left":
                            tv = tg.get_vehicle(v)
                            if lbox.contains(Point(tv.x, tv.y)):
                                temp.append(tv)
                        elif key=="front":
                            tv = tg.get_vehicle(v)
                            if fbox.contains(Point(tv.x, tv.y)):
                                temp.append(tv)

            visible_cars[key] = temp
        return visible_cars

    @staticmethod
    def get_traffic_signal_state(car):
        """Shows the traffic signals that are visible to a human driver.

        Currently included in BasicVisual as driver can 'see' the traffic signal. Later should be included in communication module.

        Args:
            car (object): vehicle object

        Returns:
            list: [color, time remaining] , if there is no signal, returns [None, None]
        """
        current_road = car.get_current_road()
        if current_road.type == "turn":
            next_node = car.get_next_node()
            current_node = car.get_current_node()
            return [None, None]
        else:
            next_node = car.get_next_node()
            current_node = car.get_current_node()
            intersection = next_node.parent
            if intersection.traffic_controller is None:
                return [None, None]
            else:

                traffic_signal = next_node.traffic_signals[car.get_lane_no()]
                return traffic_signal.get_state()

    @staticmethod
    def get_intersection_traffic(car, tg):
        """
        Used to 'peep and look' at turnings. Essentially used only in right of way rules to see if there is a car coming into the intersection from another road. Refer to the implementation docs for details on how this function works, and what the dict keys mean.

        Args:
            car (object): vehicle object
            tg (object): Traffic Graph object

        Returns:
            dict: with keys as [1,3,5,7] and values as list of vehicle objects

        Notes:
            Currently this seems to be used at all time instants so often returns dict with keys from [1,8] or [0,7] which is probably when it is called when on turn roads. Don't panic!!

        """

        next_node = car.get_next_node()
        intersection = next_node.parent
        int_nodes = intersection.get_nodes()
        index = int_nodes.index(next_node) + 1
        traffic = dict.fromkeys([1, 3, 5, 7], [])

        offset = index - 1

        traffic_zones = [1, 3, 5, 7]
        for zone in traffic_zones:
            if zone == index:

                pass
            else:
                tempind = (zone - offset) % 8   # 8 is the number of lanes
                node = intersection.get_node_by_id(zone-1)
                edges = list(tg.get_out_edges(node))
                # assert len(edges) == 1
                road = tg.get_road(edges[0])

                box = get_polygon_from_point(node.x, node.y, road.heading[-1], road.width/2, 24, -20)

                traffic[tempind] = tg.get_vehicles_by_bounding_box(bounding_box=box.bounds, objects=False)

        for key in traffic.keys():
            temp = []
            for v in traffic[key]:
                if (not tg.get_vehicle(v).is_locked()) and not (car==tg.get_vehicle(v)):

                    temp.append(tg.get_vehicle(v))
            traffic[key] = temp
        return traffic


def get_car_bounding_box(car):
    """
    Returns the bounding box (not axis aligned; generic one) of the vehicle.

    Args:
        car (object): vehicle object

    Returns:
        object: shapely Polygon object
    """

    poly = get_polygon_from_point(car.x, car.y, car.get_heading(), car.get_width() / 2, car.get_length() / 2,
                                  -car.get_length() / 2)

    return poly


def get_car_bounding_box_front(car):
    """
    Returns the bounding box (not axis aligned; generic one) of the area in the front vehicle (determined by safe dist).

    Args:
        car (object): vehicle object

    Returns:
        object: shapely Polygon object
    """

    # width = (road_c.LANE_WIDTH)/2 # + car.get_width()/2)/2
    width = car.get_width()
    front_distance = car.get_length()/2 + v_c.CAR_SAFE_DIST
    back_distance = -car.get_length()/2
    poly = get_polygon_from_point(car.x, car.y, car.get_heading(), width, front_distance, back_distance)

    return poly


def get_car_bounding_box_back(car):
    """
    Returns the bounding box (not axis aligned; generic one) of the area in the back vehicle (determined by safe dist).

    Args:
        car (object): vehicle object

    Returns:
        object: shapely Polygon object
    """

    width = (road_c.LANE_WIDTH + car.get_width()/2)/2
    front_distance = -car.get_length()/2
    back_distance = car.get_length()/2 + v_c.CAR_SAFE_DIST
    poly = get_polygon_from_point(car.x, car.y, car.get_heading(), width, front_distance, back_distance)
    return poly


def get_car_bounding_box_adj(car, lane_width, direction):
    """
    Returns a bounding box of the area in the adjacent lane of the vehicle determined by the input args.

    Args:
        car (object): vehicle object
        lane_width (float):
        direction (str): 'left' or 'right'

    Returns:
        object: shapely Polygon object
    """
    width = (road_c.LANE_WIDTH + car.get_width()/2)/2
    distance = car.get_length()/2 + v_c.CAR_SAFE_DIST
    poly = get_adj_polygon_from_point(car.x, car.y, car.get_heading(), width, distance, direction, lane_width)
    return poly


def get_adj_polygon_from_point(x, y, theta, width, distance, direction, lane_width):
    """
    Returns a polygon with lane_width away from (x, y) along theta angle in given direction with sizes distance x width.

    Args:
        x (float): x coordinate
        y (float): y coordinate
        theta (float): angle in radians
        width (float): width of the bounding box
        distance (float): length of the bounding box
        direction (str): 'left' or 'right'
        lane_width (float): to check lane_width distance away = finding in the adjacent lane

    Returns:
        object: shapely Polygon object
    """
    x_front = x + math.cos(theta) * distance
    y_front = y + math.sin(theta) * distance

    # x_back = x - math.cos(theta) * distance
    # y_back = y - math.sin(theta) * distance
    x_back = x
    y_back = y

    line = LineString([(x_back, y_back), (x_front, y_front)])
    adj_line = line.parallel_offset(lane_width, direction)
    r_line = adj_line.parallel_offset(width, 'right')
    l_line = adj_line.parallel_offset(width, 'left')

    rx, ry = r_line.coords.xy
    lx, ly = l_line.coords.xy

    poly = Polygon(((rx[0], ry[0]), (rx[1], ry[1]), (lx[0], ly[0]), (lx[1], ly[1])))

    return poly


def get_polygon_from_point(x, y, theta, width, front_distance, back_distance):
    """
    Returns a polygon around a point in a given heading angle with given dimensions. Front_distance and back_distance are determined by the heading angle to figure out which one is front or back.

    Args:
        x (float): x coordinate
        y (float): y coordinate
        theta (float): heading angle in radians
        width (float): width of the bounding box
        front_distance (float): distance from the point to the end of the front of the box (ahead of the point)
        back_distance (float): distance from the point to the end of the back of the box (behind the point)

    Returns:
        object: shapely Polygon object
    """

    x_front = x + math.cos(theta) * front_distance
    y_front = y + math.sin(theta) * front_distance

    x_back = x + math.cos(theta) * back_distance
    y_back = y + math.sin(theta) * back_distance

    line = LineString([(x_back, y_back), (x_front, y_front)])
    r_line = line.parallel_offset(width, 'right')
    l_line = line.parallel_offset(width, 'left')

    rx, ry = r_line.coords.xy
    lx, ly = l_line.coords.xy

    poly = Polygon(((rx[0], ry[0]), (rx[1], ry[1]), (lx[0], ly[0]), (lx[1], ly[1])))

    return poly


# fixme: Talk to Yin and update ctypes related code

# class Poly(Structure):
#     _fields_ = [('rx_0', c_double),
#         ('ry_0', c_double),
#         ('rx_1', c_double),
#         ('ry_1', c_double),
#         ('lx_0', c_double),
#         ('ly_0', c_double),
#         ('lx_1', c_double),
#         ('ly_1', c_double)];
#
#
# def get_adj_polygon_from_point(x, y, theta, width, distance, direction, lane_width):
#
#     libc = CDLL('core/lib/physics/util.so')
#     # libc = CDLL('~/RoadNetworks/core/lib/physics/util.so')
#     libc.get_adj_lane_safe_polygon_from_point.restype = ctypes.POINTER(Poly)
#     result_ptr = libc.get_adj_lane_safe_polygon_from_point(c_double(x), c_double(y), c_double(theta),
#                                                            c_double(width), c_double(distance),
#                                                            c_char_p(bytes(direction, 'utf-8')), c_double(lane_width))
#     result = result_ptr.contents
#
#     poly = Polygon(((result.rx_0, result.ry_0), (result.rx_1, result.ry_1), (result.lx_1, result.ly_1),
#                     (result.lx_0, result.ly_0)))
#
#     return poly
#
#
# def get_polygon_from_point(x, y, theta, width, front_distance, back_distance):
#
#     libc = CDLL('core/lib/physics/util.so')
#     # libc = CDLL('~/RoadNetworks/core/lib/physics/util.so')
#     libc.get_lane_safe_polygon_from_point.restype = ctypes.POINTER(Poly)
#     result_ptr = libc.get_lane_safe_polygon_from_point(c_double(x), c_double(y), c_double(theta),
#                                                        c_double(width), c_double(front_distance),
#                                                        c_double(back_distance))
#
#     result = result_ptr.contents
#     poly = Polygon(((result.rx_0, result.ry_0), (result.rx_1, result.ry_1), (result.lx_1, result.ly_1),
#                     (result.lx_0, result.ly_0)))
#
#     return poly
