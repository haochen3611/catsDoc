# -*- coding: utf-8 -*-
"""The maps modules consists of several classes that cater to different details of the urban landscape.

Map module is central to the effectiveness of CATS. The most important requirements/features of maps are

    - Maps should represent an urban traffic scenario, along with the landscape of the city.
    - Maps require a detailed modeling of roads, intersections, lanes, etc.
    - Maps should encode the topological connections between roads, for routing.
    - Specific to CATS, maps also store the spatial details of all the vehicles in the simulation.

"""
from rtree import index
import networkx as nx
import itertools
import math
import numpy as np
from core.lib.maps.utils import constants as map_c
from shapely.geometry import Polygon, LineString, Point
from core.lib.infrastructure.traffic import TrafficController
from core.helpers import norm, perpendicular_distance_of_point_from_line
# from core.lib.maps.generate_traffic_graph import create_sample_network
from shapely.affinity import rotate
import matplotlib.pyplot as plt
from random import choice


def get_grade(x, y):
    """Determines the landscape of the region under the map.

    This method is determined by a smooth function, and returns the function value evaluated at the input location. The function must have subtle variations in the grade with relative changes in x and y. This function is now external to the map, but later it can be integrated into the TrafficGraph class.

    Args:
        x (float): x coordinate 
        y (float): y coordinate

    Returns:
        float: grade of the (x,y) coordinate
    
    Todo: 
        Determine a 2-D function which maps to the grade (altitude) of a given point.
    """
    
    return x**2+y**2


class SubNode:
    """Characterizes points of a line string along a curved road.

    """

    def __init__(self, point_x, point_y):
        """
        Initializes the coordinates and the altitude (grade) of the point.

        Args:
            point_x (float): x coordinate
            point_y (float): y coordinate
        """
        self.__x = point_x
        self.__y = point_y
        self.__grade = get_grade(point_x, point_y)

    @property
    def x(self):
        return self.__x

    def set_x(self, x):
        self.__x = x

    @property
    def y(self):
        return self.__y

    def set_y(self, y):
        self.__y = y

    def get_grade(self):
        return self.__grade

    def set_grade(self, g):
        self.__grade = g


class RoadString:
    """Encapsulates all the information about the road.

    Attributes:
        __sub_nodes (list): list of SubNode objects at points on the linestring
        __no_sub_nodes (int): number of subnodes in the linestring
        __segment_headings (list): list of heading angle from one SubNode on the line string to the next SubNode
        __perpendicular_segments (list): list of lines perpendicular to the line joining two adjacent SubNodes
        __length_segments (list): list of lengths of segments joining two adjacent SubNodes

    """

    def __init__(self, line_string):
        """
        Constructor for class RoadString.

        Args:
            line_string (shapely LineString): line string of the road
        """
        self.__sub_nodes = None
        self.__line_string = line_string
        # An osm road has a line string of length n. sub_nodes will be of length n-1 (excluding the first node).
        self.__perpendicular_segments = None
        # n-1 segments
        # not actually perpendicular though
        self.__segment_headings = None
        # n-1 segment headings
        self.__no_sub_nodes = 0
        self.__length_segments = None
        self.set_attributes(line_string)

    @property
    def line_string(self):
        return self.__line_string

    def set_attributes(self, line_string):
        """
        Calculates all the information of the road from the given line string.

        Args:
            line_string (shapely LineString): of the road

        """

        x, y = line_string.coords.xy
        self.__sub_nodes = [SubNode(x[i], y[i]) for i in range(1, len(x))]
        self.__segment_headings = [math.atan2(y[i]-y[i-1], x[i]-x[i-1]) for i in range(1, len(x))]
        self.__perpendicular_segments = [self.get_perpendicular_segment(
            self.__segment_headings[i], self.__segment_headings[i+1], (x[i], y[i]))
            for i in range(len(self.__segment_headings)-1)]
        self.__perpendicular_segments.append(self.get_perpendicular_segment(
            self.__segment_headings[len(x)-2], self.__segment_headings[len(x)-2], (x[-1], y[-1])))
        self.__no_sub_nodes = len(self.__sub_nodes)
        self.__length_segments = [norm((x[i]-x[i-1], y[i]-y[i-1])) for i in range(1, len(x))]

    @staticmethod
    def get_perpendicular_segment(theta1, theta2, point):
        """
        Calculates equation of a line passing through a point with slope as an average of two slopes.

        Args:
            theta1 (float): angle in radians
            theta2 (float): angle in radians
            point (tuple): (x,y) coordinates of the point

        Returns:
            tuple: (1, -slope, -intercept) where the equation of line is :math: `y + slope * x + intercept = 0`
        """
        x, y = point
        # if (theta1 + theta2)/2 == math.pi/2 or (theta1 + theta2)/2 == math.pi/2):

        slope = math.tan((theta1 + theta2) / 2)
        if slope > 1e3:
            return 0, 1, -x
        else:
            c = y - slope * x
        return 1, -slope, -c

    @property
    def subnodes(self):
        return self.__sub_nodes

    @property
    def perpendicular_segments(self):
        return self.__perpendicular_segments

    @property
    def no_sub_nodes(self):
        return self.__no_sub_nodes

    @property
    def segment_headings(self):
        return self.__segment_headings

    @property
    def segment_lengths(self):
        return self.__length_segments


class Node:
    """
    Class determining the data structure of a Node (end points of a Road).

    Attributes:
        __x (float): x coordinate
        __y (float): y coordinate
        __grade (float): altitude
        __parent (class): Intersection of which the node is a part of
        __id (int): identifier of the Node
        __traffic_signals (object) : Traffic Signal on the node (if any)
        __incoming (bool): If the Node ends at an intersection (it is incoming into the intersection)

    Notes:
            This class is similar to a (class) SubNode, except that (class) Node is an endpoint of a (class) Road and
            usually is a bound to (class) Intersection.

    """
    def __init__(self, point_x, point_y, incoming, _id=None, parent=None, traffic_signal=None):
        """
        Constructor for class Node.

        Args:
            point_x (float): x coordinate
            point_y (float): y coordinate
            incoming (bool): if the road ends / starts at this node
            _id (int): unique identifier wrt the TrafficGraph
            parent (object): parent Intersection object
            traffic_signal (object): traffic signal object
        """
        self.__x = point_x
        self.__y = point_y
        self.__grade = get_grade(point_x, point_y)
        self.__parent = parent
        self.__id = _id
        self.__traffic_signals = None
        self.__incoming = incoming
        if traffic_signal is None:
            self.__traffic_signals = dict().fromkeys(range(3), None)
        else:
            self.__traffic_signals = dict().fromkeys(range(3))

    def __str__(self):
        return str(self.__id)

    @property
    def id(self):
        return self.__id

    @property
    def parent(self):
        return self.__parent

    @property
    def x(self):
        return self.__x

    def set_x(self, x):
        self.__x = x

    @property
    def y(self):
        return self.__y

    def set_y(self, y):
        self.__y = y

    def get_grade(self):
        return self.__grade

    def set_grade(self, g):
        self.__grade = g

    def get_id(self):
        return self.__id

    def set_id(self, _id):
        self.__id = _id

    def is_incoming(self):
        return self.__incoming

    @property
    def traffic_signals(self):
        return self.__traffic_signals


class Intersection:
    """
    Characterizes a traffic junction.

    Attributes:
        __x (float): x coordinate
        __y (float): y coordinate
        __grade (float): altitude
        __osmid (int): unique identifier of the Intersection
        __traffic_controller (object) : TrafficController on the Intersection
        __nodes (list): list of Node objects

    Notes:
        An intersection contains multiple Nodes which are the end points of the roads leading upto the intersection. An intersection is initialized with 0 Nodes. Nodes are added later while initializing a map using `add_node` method.
        
        The present implementation assumes the following.
            - Fixed number of nodes. Currently only support a '+' kind of intersection (a four way junction)
            - Intersections are axis aligned.

    """
    #Todo: Extend to dynamically create an intersection with variable number of roads coming in (different shapes)
    
    #Todo: This extension is coupled in a way with extending TrafficController to take variable size signals.

    def __init__(self, point_x, point_y, osmid=None, traffic_controller=None):
        """
        Constructor for class Intersection.

        Args:
            point_x (float): x coordinate
            point_y (float): y coordinate
            osmid (int): unique identifier of the intersection
            traffic_controller (object): TrafficController object that controls the signals on the Nodes
        """
        self.__x = point_x
        self.__y = point_y
        self.__grade = get_grade(point_x, point_y)
        self.__osmid = osmid
        self.__type = None
        self.__nodes = []
        self.__traffic_controller = traffic_controller
        self.__no_of_nodes = 0

    def get_x(self):
        return self.__x

    def set_x(self, x):
        self.__x = x

    def get_y(self):
        return self.__y

    def set_y(self, y):
        self.__y = y

    def get_grade(self):
        return self.__grade

    def set_grade(self, g):
        self.__grade = g

    def get_osmid(self):
        return self.__osmid

    def set_osmid(self, osmid):
        self.__osmid = osmid

    def get_nodes(self):
        return self.__nodes

    def get_node_by_id(self, _id):
        return self.__nodes[_id]

    def add_node(self, node):
        self.__nodes.append(node)
        self.__no_of_nodes += 1

    @property
    def traffic_controller(self):
        return self.__traffic_controller

    def get_no_of_nodes(self):
        return self.__no_of_nodes


class Lane:
    """Characterizes a lane that is a part of a Road.

    Attributes:
        __vehicles (list) : list of vehicle objects
        __lane_no (int) : lane_no is calculated from 0 (center of road, i.e. left most lane)
        __lane_type (str) : type of lane (turn only etc)
        __road (object) : Road of which the lane is a part of
        __lane_polygon (object) : Shapely bounding polygon of the lane
        __lane_center (object) : Shapely LineString object of the lane_center
        __speed_limit (float) : speed limit of the road
        __name (str) : name oft he lane
        __start_x (float) : x coordinate of the start of the lane center
        __start_y (float) : y coordinate of the start of the lane center
        __end_x (float) : x coordinate of the end of the lane center
        __end_y (float) : y coordinate of the end of the lane center

    Notes:
        Current Implementation (to be deprecated): Provides the bare minimum information such as lane_center (used to calculated distance from lane center), lane_no, speed_limit etc. Has no particularly important role other than the attributes defined.

        Future Implementation (under development): The vehicles are store on the lanes. If a vehicle changes a lane, a method on lanes is called to update the vehicles on the source lane and the destination lane. The vehicles on a lane are stored in a linked list in order of decreasing(?) distance from the end of the road.

    """

    def __init__(self, lane_no, road, lane_polygon, lane_center, speed_limit=None, _type=None):
        """
        Constructor for class Lane.

        Args:
            lane_no (int): lane_no is calculated from 0 (center of road, i.e. left most lane)
            road (object): Road of which the lane is a part of
            lane_polygon (object): Shapely bounding polygon of the lane
            lane_center (object): Shapely LineString object of the lane_center
            speed_limit (float): speed limit of the road
            _type (str): type of lane (turn only etc)

        """
        self.__vehicles = []
        self.__lane_no = lane_no
        self.__lane_type = _type
        self.__road = road

        self.__lane_polygon = lane_polygon
        self.__lane_center = lane_center
        self.__speed_limit = speed_limit if speed_limit is not None else road.speed_limit
        self.__name = str(self.__road.name)+"_" + str(self.__lane_no)
        x, y = self.__lane_center.coords.xy

        self.__start_x = x[0]
        self.__start_y = y[0]
        self.__end_x = x[-1]
        self.__end_y = y[-1]

    def __str__(self):
        return self.__name

    @property
    def lane_no(self):
        return self.__lane_no

    @property
    def road(self):
        return self.__road

    @property
    def lane_polygon(self):
        return self.__lane_polygon

    @property
    def lane_center(self):
        return self.__lane_center

    @property
    def speed_limit(self):
        return self.__speed_limit

    @property
    def lane_center_xy(self):
        return [self.__start_x, self.__start_y, self.__end_x, self.__end_y]

    def remove_vehicle(self, vehicle):
        """
        Notes:
            Not used currently. To be used in the future

        Args:
            vehicle (object): Vehicle that needs to be removed from the lane

        Raises:
            ValueError: Vehicle not found in the lane

        """

        try:
            self.__vehicles.remove(vehicle)
        except ValueError:
            print("vehicle not found in lane.__vehicles")

    def get_vehicles(self):

        return self.__vehicles


class Road:
    """Characterizes the Road in a TrafficGraph.

    The Road is determined by a start node, an end node, a RoadString (for the curvature/ linestring). It has attributes like speed limit, number of lanes etc that are specified while constructing the Road, and attributes like width, polygons that are calculated.

    Notes:
        There are two types of roads,
        1) normal road that connects two nodes (which are on two different intersections),
        2) turn road that connects two nodes within the same intersection. In this setting, the vehicles can be
        simulated on a micro scale within an intersection (traffic junction which is usually atleast 20 meters in
        diameter). By considering two types of roads, there are several edge cases on the vehicle class that need to be
        taken care of when trying to update the state of the vehicle. Also, the current implementation is only for a
        plus shaped intersection, but when there are various types of intersections, care must be taken in assigning
        the turn roads.

    Attributes:
        __start_node (object) : start_node
        __end_node (object) : end_node
        __type (str) : _type
        __speed_limit (float) : speed limit of the road
        __line_string (object) : Shapely LineString of the center of the road
        __name (str) : name of the road
        __osmid (int) : unique identifier of the Road
        __no_of_lanes (int) : no_of_lanes
        __lane_width (float) : lane_width
        __left_line_string (object) : Shapely LineString of the left edge of the road.
        __left_line_string_x (list ?) : x coords of the left LineSting
        __left_line_string_y (list ?) : y coords of the left LineString
        __road_string (object): RoadString that determines the curvature, heading of the road
        __length (float) : length of the road (measured along the line string)
        __width (float) : no_of_lanes* lane_width
        __lanes (list) : list of the Lanes on the Road
        __heading (list) : list of the headings across difference segments in the RoadString
        __traffic_count (int) : number of vehicles on the road
        __traffic_trace (dict) : history of traffic attributes on the road
        __lane_wise_traffic (dict) : traffic attributes
        __total_traffic (dict) : traffic attributes
        __time (float) : estimated travel time along a road (determined by the speed limit)
        __avg_time (float) : estimated travel time along a road (average of the time taken by the traffic on that road)
        __val :
        __min_bounding_box (object) : Shapely Polygon of the axis aligned polygon of the Road
        __bounding_polygon (object) : Shapely Polygon of the Road (the actual polygon, which fits the edges)

    """

    def __init__(self, start_node, end_node, line_string, _type='straight', no_of_lanes=map_c.NO_OF_LANES, name=None,
                 osmid=None, lane_width=map_c.LANE_WIDTH, speed_limit=40):
        """
        Constructor for class Road.

        Args:
          start_node (object): start_node
          end_node (object) : end_node
          _type (str) : _type
          speed_limit (float) : speed limit of the road
          line_string (object) : Shapely LineString of the center of the road
          osmid (int) : unique identifier of the Road
          no_of_lanes (int) : no_of_lanes
          lane_width (float) : lane_width
          name (str) : name of the road

        """
        self.__start_node = start_node
        self.__end_node = end_node
        self.__type = _type
        self.__speed_limit = speed_limit * choice([0.7, 1, 1.3])
        self.__line_string = line_string
        self.__left_line_string = line_string.parallel_offset(no_of_lanes* lane_width/2, 'right')
        x,y = self.__left_line_string.xy
        self.__left_line_string_x = x
        self.__left_line_string_y = y
        self.__road_string = RoadString(line_string)
        self.__length = sum(self.__road_string.segment_lengths)
        self.__name = name
        self.__osmid = osmid
        self.__no_of_lanes = no_of_lanes
        self.__lane_width = lane_width
        self.__width = no_of_lanes* lane_width
        self.__lanes = []
        # self.__lanes = [Lane(lane_no=i, road=self) for i in range(no_of_lanes)]
        self.__heading = self.road_string.segment_headings
        self.__traffic_count = 0
        self.__traffic_trace = []
        self.__lane_wise_traffic = {'count': 0, 'cars': [], 'average_velocity': 0, 'variance_velocity': 0, 'velocities': []}
        self.__total_traffic = {'count': 0, 'vehicles': [], 'average_velocity': 0, 'variance_velocity': 0, 'velocities': []}
        self.__time = self.__length/self.__speed_limit
        self.__avg_time = self.__time
        self.__val = 0
        self.__min_bounding_box = None
        self.__bounding_polygon = None
        self.__name = str(self.__start_node.id) + "-" + str(self.__end_node.id) + "-msp-" + str(self.__speed_limit)
        self.__lane_polygons()

    def __str__(self):
        return self.__name

    def __lane_polygons(self):
        """
        Computes the Shapely Polygon objects of the lanes on the Road.

        Notes:
            The current implementation assumes straight roads (where the linestring/RoadString determines just the
            straight line joining the start node and end node.
            Future implementations must modify this method to suit curved roads.

        """

        center_linestring = self.__line_string

        if self.__no_of_lanes % 2 == 0:
            for ind in range(1, 1 + self.__no_of_lanes//2):
                distance = self.__lane_width/2 + self.__lane_width * (self.__no_of_lanes/2 - ind)
                left_linestring = center_linestring.parallel_offset(distance, 'left')
                poly = polygon_from_linestring(left_linestring, self.__lane_width/2)
                lane = Lane(lane_no=ind-1, road=self, lane_polygon=poly, lane_center=left_linestring)
                self.__lanes.append(lane)

            for ind in range(self.__no_of_lanes//2):
                distance = self.__lane_width/2 + self.__lane_width * ind
                right_linestring = center_linestring.parallel_offset(distance, 'right')
                poly = polygon_from_linestring(right_linestring, self.__lane_width/2)
                lane = Lane(lane_no=self.__no_of_lanes//2 + ind,
                            road=self, lane_polygon=poly, lane_center=right_linestring)

                self.__lanes.append(lane)

        else:
            for ind in range(1, 1 + self.__no_of_lanes // 2):
                distance = self.__lane_width * (1 + self.__no_of_lanes // 2 - ind)
                # left_linestring = center_linestring.parallel_offset(distance, 'left')
                right_linestring = center_linestring.parallel_offset(distance, 'right')
                x, y = right_linestring.coords.xy
                right_linestring = LineString([(x[-1], y[-1]), (x[0], y[0])])

                poly = polygon_from_linestring(right_linestring, self.__lane_width / 2)
                lane = Lane(lane_no=ind-1, road=self, lane_polygon=poly, lane_center=right_linestring)

                self.__lanes.append(lane)

            poly = polygon_from_linestring(center_linestring, self.__lane_width / 2)
            lane = Lane(lane_no=self.__no_of_lanes//2,
                        road=self, lane_polygon=poly, lane_center=center_linestring)

            self.__lanes.append(lane)

            # issue: Shapely issue #284: parallel offset to the right returns a line string in the reverse order.
            # Did not find a quick fix, so manually reversing it after the parallel offset

            for ind in range(self.__no_of_lanes // 2):
                distance = self.__lane_width + self.__lane_width * ind
                left_linestring = center_linestring.parallel_offset(distance, 'left')
                poly = polygon_from_linestring(left_linestring, self.__lane_width / 2)
                lane = Lane(lane_no=self.__no_of_lanes//2 + ind + 1,
                            road=self, lane_polygon=poly, lane_center=left_linestring)

                self.__lanes.append(lane)

    @property
    def name(self):
        return self.__name

    @property
    def start_node(self):
        return self.__start_node

    @property
    def end_node(self):
        return self.__end_node

    @property
    def width(self):
        return self.__width

    @property
    def type(self):
        return self.__type

    @property
    def val(self):
        return self.__val

    @property
    def avg_time(self):
        return self.__avg_time

    @property
    def min_bounding_box(self):
        polygon = self.bounding_polygon
        return polygon.bounds

    @property
    def road_string(self):
        return self.__road_string

    @property
    def lanes(self):
        return self.__lanes

    @property
    def lane_width(self):
        return self.__lane_width

    @property
    def no_of_lanes(self):
        return self.__no_of_lanes

    @property
    def road_length(self):
        return self.__length

    @property
    def line_string(self):
        return self.__line_string

    @property
    def left_line_string(self):
        return self.__left_line_string

    @property
    def road_string(self):
        return self.__road_string

    @property
    def heading(self):
        return self.__heading

    @property
    def speed_limit(self):
        return self.__speed_limit

    @property
    def bounding_polygon(self):
        """
        Notes:
            The current implementation only considers straight roads (roads which are not curved). Future
            implementations must modify this method to take the curvature/turns into consideration.
            Note that the return type also changes when curved roads are considered.

        Returns:
            list: of the points (of the polygon)
        """

        # The bounding polygon for turn roads are not consistent. The polygon partly overlaps with the adjacent straight
        # roads. For now this is fine, but this should be corrected later.
        # future: currently assumes heading[0]. need to extend to polygon of every segment
        # theta = math.atan2((self.end_node.Y-self.start_node.Y), (self.end_node.X-self.start_node.X))
        theta = self.__heading[0]
        w = (self.__lane_width * self.__no_of_lanes)/2

        pt1 = (self.start_node.x - w * math.sin(theta), self.start_node.y + w * math.cos(theta))
        pt2 = (self.start_node.x + w * math.sin(theta), self.start_node.y - w * math.cos(theta))

        theta = math.atan2(-(self.end_node.y-self.start_node.y), -(self.end_node.x-self.start_node.x))
        pt3 = (self.end_node.x - w * math.sin(theta), self.end_node.y + w * math.cos(theta))
        pt4 = (self.end_node.x + w * math.sin(theta), self.end_node.y - w * math.cos(theta))

        # polygon = Polygon([pt1, pt2, pt3, pt4])

        return [pt1, pt2, pt3, pt4]

    def reset_traffic(self):
        """
        Assigns the traffic attributes to the class Attributes at the end of the time interval and resets the
        traffic attributes.

        Notes:
            Future research may include a time series data of the traffic attributes.

        """
        eps = 0.1
        self.__val = len(self.__total_traffic['velocities'])
        if len(self.__total_traffic['velocities']) > 0:
            self.__avg_time = self.__length/(eps + self.__total_traffic['average_velocity'])
        else:
            self.__avg_time = self.__time
        self.__total_traffic = {'count': 0, 'vehicles': [], 'average_velocity': 0, 'variance_velocity': 0,
                                'velocities': []}

    def update_traffic_from_vehicle_data(self, velocity):
        """
        Does the job of a loop detector by updating the attributes of the traffic, like number of vehicles, average
        velocity etc. This update is done one vehicle at a time. Each vehicle, after updating its states, calls this
        method to update the attributes of the road on which it is travelling.

        Args:
            velocity (float) : velocity of the vehicle calling this method

        """

        count = self.__total_traffic['count']
        self.__total_traffic['velocities'].append(velocity)
        self.__total_traffic['average_velocity'] = \
            (self.__total_traffic['average_velocity']*count + velocity)/(count+1)
        self.__total_traffic['count'] += 1

    def get_lane(self, x, y, segment=0):
        """
        Calculates the lane number of the vehicle by taking its coordinates (x,y). The distance from the left edge of
        the road is calculated and divided by the lane width to find the lane number.

        Notes:
            When curved roads are implemented in the future, this method does not have to change much. If the
            __left_line_string attribute is modified for curved roads, this method only needs to use the segment number
            and calculate the distance (by using the line string in that segment).

        Args:
            x (float): x coordinate of the vehicle
            y (float): y coordinate of the vehicle
            segment (int): optional (for now, until this method is updated for curved roads)

        Returns:
            int: lane number calculated

        """

        # fixme: update this method for curved roads
        point = Point(x,y)

        llsx = self.__left_line_string_x
        llsy = self.__left_line_string_y

        lx, ly = self.line_string.xy

        slope = math.atan2(llsy[1]-llsy[0], llsx[1]-llsx[0])
        c = llsy[1] - math.tan(slope)*llsx[1]
        if abs(c)>1e3:
            left_edge_distance = abs(llsx[1] - x)
        else:
            left_edge_distance = abs(y - math.tan(slope)*x - c)/math.sqrt(math.tan(slope)**2 + 1)

        # left_edge_distance = point.distance(self.__left_line_string)

        lane_no = int(left_edge_distance//self.lane_width)
        return min(lane_no, self.no_of_lanes-1)

        # distance = self.get_distance_from_road_center(x, y)
        # road_theta = self.__heading[segment]
        # theta = math.atan2((self.__end_node.y - y), (self.__end_node.x - x))
        # lane = None
        # if self.__no_of_lanes % 2 == 1:
        #     for ind in range(1 + self.__no_of_lanes//2):
        #         if distance < (self.__lane_width/2.0 + self.__lane_width * ind):
        #             if ind == 0:
        #                 lane = self.__no_of_lanes//2
        #             else:
        #                 if road_theta == math.pi:
        #                     if math.pi - theta < road_theta:
        #                         lane = self.__no_of_lanes//2 + ind
        #                     else:
        #                         lane = self.__no_of_lanes//2 - ind
        #                 else:
        #                     if theta < road_theta:
        #                         lane = self.__no_of_lanes//2 + ind
        #                     else:
        #                         lane = self.__no_of_lanes//2 - ind
        #             break
        #     if lane == None:
        #         lane=0
        #         #raise LookupError
        # else:

        #     for ind in range(1, 1+self.__no_of_lanes//2):
        #         if distance < self.__lane_width * ind:
        #             if road_theta == math.pi:
        #                 if math.pi - theta < road_theta:
        #                     lane = self.__no_of_lanes//2 + ind - 1
        #                 else:  # inner edge of the road
        #                     lane = self.__no_of_lanes//2 - ind
        #                 break
        #             else:
        #                 if theta < road_theta:              # outer edge of the road
        #                     lane = self.__no_of_lanes//2 + ind - 1
        #                 else:                                   # inner edge of the road
        #                     lane = self.__no_of_lanes//2 - ind
        #                 break

        # return lane_no

    def get_distance_from_road_center(self, x, y):
        """
        calculates the distance from the center of the road using the __line_string

        Args:
            x (float): x coordinate
            y (float): y coordinate

        Returns:
            float: distance from the center of the Road

        """
        line = self.__line_string
        point = Point(x, y)
        distance = point.distance(line)
        return distance


class TrafficGraph:
    """Comprises of and defines the topological connections between the other classes defined in map module.

    TrafficGraph defines the landscape, the connections between roads, and everything related to the map used in the
    simulation. TrafficGraph currently uses a graph data structure to store the topological connections, and an
    rtree data structure to store the spatial information related to the static objects (nodes, intersections, roads)
    and mobile objects (such as the vehicles).

    Attributes:
        __graph (object): a networkx graph object
        _simulation (object): simulation object that uses the trafficgraph
        __intersections_table (dict): a dictionary of the intersections defined in the trafficgraph
        __intersections_count (int): len of the __intersections_table
        __sources (list): list of the nodes in the graph that have 0 incoming edges
        __sinks (list) : list of the nodes in the graph that have 0 outgoing edges
        __nodes_count (int) : number of nodes in the graph

        __roads_count (int) :
        __roads_table : dict()
        __roads_tree : index.Index()
        __roads_tree.interleaved : True

        __vehicles_count : 0
        __vehicles_table : {}
        __vehicles_tree : index.Index()
        __vehicles_tree.interleaved : True

    """
    def __init__(self, simulation):
        """
        Constructor for class TrafficGraph.

        Args:
            simulation (object): simulation object that uses the TrafficGraph class
        """
        self.__graph = nx.DiGraph()
        self._simulation = simulation

        self.__intersections_table = dict()
        self.__intersections_count = 0
        self.__sources = []
        self.__sinks = []
        self.__nodes_count = 0

        self.__roads_count = 0
        self.__roads_table = dict()
        self.__roads_tree = index.Index()
        self.__roads_tree.interleaved = True

        self.__vehicles_count = 0
        self.__vehicles_table = {}
        self.__vehicles_tree = index.Index()
        self.__vehicles_tree.interleaved = True

    def create_sample_network(self):
        """
        Creates a default network (or chooses from among a set of default networks).

        Notes:
            Must include a GUI support to enable a user to create a network on the fly by dragging and dropping
        """

        self.__graph, self.__intersections_table = build_sample_network()

        # for node in self.__graph.nodes():
        #     if self.__graph.out_degree(node) == 0:
        #         self.__sinks.append(self.get_node_attribute(node))
        #     if self.__graph.in_degree(node) == 0:
        #         self.__sources.append(self.get_node_attribute(node))

        self.__build_rtree()

        # for source in [9, 25, 27, 43, 45, 61, 77, 79, 95, 89]:
        # for source in [1, 3, 11, 19, 27, 29]:
        for source in [11, 19, 108, 116, 33, 65, 63, 95]:
            self.__sources.append(self.get_node_attribute(source))
        for sink in [104, 112, 120, 126, 128]:
        # for sink in [126, 128]:
            self.__sinks.append(self.get_node_attribute(sink))

        self.__intersections_count = len(self.__intersections_table.keys())
        self.__roads_count = len(self.__roads_table.keys())
        # for _, i in self.__intersections_table.items():
        #     self.__nodes_count += i.get_no_of_nodes()

    def __build_rtree(self):
        """
        Builds an rtree using the Nodes from the __graph attribute. Since building an rtree for the Nodes is a one time
        event, we are not using a bulk loader to generate the rtree. In the case of rtree storing vehicle (or any mobile)
        objects that have to constantly deleted and updated, we use a bulk method to build an rtree from a generator.

        """

        for node in self.__graph.nodes(data='object'):
            # node = [id, object]
            temp_id = node[1].id
            temp_x = node[1].x
            temp_y = node[1].y
            self.__roads_tree.insert(temp_id, (temp_x, temp_y, temp_x, temp_y))
            self.__nodes_count += 1

    def get_nearest_nodes(self, bounding_box, number, generator=True, objects=False):
        """

        Args:
            bounding_box (tuple): axis aligned bounding box coordinates (xmin, ymin, xmax, ymax)
            number (int): number of required nodes
            generator (bool): If the output should be a generator
            objects (bool): If the output should be node ids or objects

        Returns:
            list: (or generator) of the given number of nodes closest to a bounding box ordered by distance

        References:
            http://toblerity.org/rtree/tutorial.html

        """

        if generator:
            return self.__roads_tree.nearest(bounding_box, number, objects)
        else:
            return list(self.__roads_tree.nearest(bounding_box, number, objects))

    def get_nodes_by_bounding_box(self, bounding_box, generator=True, objects=False):
        """
        Returns the nodes in a given axis aligned bounding box.

        Args:
            bounding_box (tuple): axis aligned bounding box coordinates (xmin, ymin, xmax, ymax)
            generator (bool): If the output should be a generator
            objects (bool): If the output should be node ids or objects

        Returns:
            list: (or generator) of the nodes closest to a bounding box ordered by distance

        References:
            http://toblerity.org/rtree/tutorial.html

        """

        if generator:
            return self.__roads_tree.nearest(bounding_box, objects)
        else:
            return list(self.__roads_tree.nearest(bounding_box, objects))

    def get_nearest_vehicles(self, bounding_box, number, generator=True, objects=False):
        """
        Returns the vehicles in a given axis aligned bounding box.

        Args:
            bounding_box (tuple): axis aligned bounding box coordinates (xmin, ymin, xmax, ymax)
            number (int): number of required vehicles
            generator (bool): If the output should be a generator
            objects (bool): If the output should be vehicle ids or objects

        Returns:
            list: (or generator) of the given number of vehicles closest to a bounding box ordered by distance

        References:
            http://toblerity.org/rtree/tutorial.html

        """

        if generator:
            return self.__vehicles_tree.nearest(bounding_box, number, objects)
        else:
            return list(self.__vehicles_tree.nearest(bounding_box, number, objects))

    def get_vehicles_by_bounding_box(self, bounding_box, generator=False, objects=False):
        """
        Returns the vehicles in a given axis aligned bounding box.

        Args:
            bounding_box (tuple): axis aligned bounding box coordinates (xmin, ymin, xmax, ymax)
            generator (bool): If the output should be a generator
            objects (bool): If the output should be vehicle ids or objects

        Returns:
            list: (or generator) of the given number of vehicles closest to a bounding box ordered by distance

        References:
            http://toblerity.org/rtree/tutorial.html

        """

        if generator:
            return self.__vehicles_tree.intersection(bounding_box, objects)
        else:
            return list(self.__vehicles_tree.intersection(bounding_box, objects))

    def get_all_nodes(self, objects=True):
        """
        Returns all the nodes stored in the graph.

        Args:
            objects (bool): If objects are yielded or object ids

        Yields:
            object: Nodes in the graph

        """
        if objects:
            for node in self.__graph.nodes():

                yield self.__graph.node[node]['object']

        # if objects:
        #     return self.__graph.nodes(data='object')
        # else:
        #     return self.__graph.nodes()

    def get_shortest_path(self, source, destination, weight='distance'):
        """
        Returns the shortest path from source to destination.

        Args:
            source (int): source node id
            destination (int): destination node id
            weight (str): parameters based on which the cost is calculated

        Returns:
            list: a list of the node ids along the shortest path from the source to the destination
        """

        # if (source not in self.__graph) or (destination not in self.__graph):

        #     return None
        # if nx.has_path(self.__graph, source, destination):
        # else:
        #     return None
        # return nx.dijkstra_path(self.__graph, source, destination, weight=weight)
        return nx.shortest_path(self.__graph, source, destination, weight=weight)

    def get_edges(self):
        """
        Returns a EdgeView containing edge. For attributes call getEdgeAttr(edge)

        Returns:

        """
        return self.__graph.edges()

    def get_road(self, edge):
        """

        Args:
            edge (tuple): [start node id, end node id]

        Returns:
            object: Road object

        """

        return self.__graph.edges[edge]['object']

    def get_roads(self):
        """

        Returns:
            list: [start node id, end node id, road object]

        """
        return self.__graph.edges(data=True)

    def get_out_edges(self, node):

        """

        Args:
            node (int): id of the node

        Returns:
            list: list of edges (which is a tuple of nodes) that come into the given node

        """
        if isinstance(node, Node):
            return self.__graph.out_edges(node.id)
        elif type(node) == int:
            return self.__graph.out_edges(node)

    def get_incoming_edges(self, node):

        """

        Args:
            node (int): id of the node

        Returns:
            list: list of edges (which is a tuple of nodes) that come into the given node

        """
        return self.__graph.in_edges(node)

    def get_adj_nodes(self, node):

        return self.__graph.adj[node]

    def get_node_count(self):

        return self.__nodes_count

    def get_vehicle_count(self):

        return len(self.__vehicles_table.keys())
        # return self.__vehicle_count

    def get_road_network_bbox(self):

        return self.__roads_tree.bounds

    def get_vehicle_tree_bbox(self):
        try:
            return self.__vehicles_tree.bounds
        except:

            return "sorry"

    def get_intersections_table(self):
        return self.__intersections_table

    def get_intersections(self):

        for _, value in self.__intersections_table.items():
            yield value

    def get_edge_attributes(self, edge):

        """
        Returns a Road object.

        Args:
            edge (tuple): [start node id, end node id]

        Returns:
            object: Road object

        """
        return self.__graph.edges[edge]

    def update_road_attributes(self, road):
        """

        Args:
            road (list): [start_node_id, end_node_id, road_object]

        Returns:
            float: estimated average time to travel on the road

        """

        self.__graph.edges[(road[0],road[1])]['time'] = road[2]['object'].avg_time
        return road[2]['object'].avg_time

    def get_node_attribute(self, node):
        """
        Returns a node object given an id

        Args:
            node(int): id of the node

        Returns:
            object: Node object

        """
        return self.__graph.nodes[node]['object']

    def get_sources(self):
        """

        Returns:
            list: of Node objects with 0 incoming edges

        """
        return self.__sources

    def get_sinks(self):
        """

        Returns:
            list: of Node objects with zero outgoing edges

        """

        return self.__sinks

    def add_vehicle(self, vehicle):
        """
        Takes a vehicle object and adds it to the traffic graph.

        Args:
            vehicle (object): vehicle object to be added (not the id)

        """

        self.__vehicles_count += 1

        self.__vehicles_table[vehicle.id] = vehicle
        # self.__vehicles_tree.insert(vehicle.ID, (vehicle.x, vehicle.y, vehicle.x, vehicle.y))

    def get_all_vehicles(self):
        """
        Returns a list of all the vehicle objects.

        Returns:
            list: list of all the vehicle objects
        """
        return self.__vehicles_table.values()

    def get_vehicles(self):
        """
        Yields the vehicle object from the __vehicles_table.

        Yields:
            object: Vehicle object
        """

        for _, value in self.__vehicles_table.items():
            yield value

    def get_vehicle(self, vehicle_id):
        """
        Returns the vehicle object with the given id in the __vehicles_table.

        Args:
            vehicle_id (int): vehicle id to be retrieved

        Returns:
            object: Vehicle object with the given vehicle_id

        """

        return self.__vehicles_table[vehicle_id]

    def remove_vehicle(self, vehicle):
        """
        Removes the given vehicle id from the __vehicles_table

        Args:
            vehicle (int): vehicle id to be deleted

        """
        self.__vehicles_count -= 1
        # self.__vehicles_tree.delete(vehicle.ID, (vehicle.x, vehicle.y, vehicle.x, vehicle.y))
        del self.__vehicles_table[vehicle.id]

    def vehicle_generator(self):
        """

        Yields:
            tuple: (id, (x,y,x,y)) id of vehicle and coordinates of vehicle

        """

        for id_, vehicle in self.__vehicles_table.items():
            yield (id_, (vehicle.x, vehicle.y, vehicle.x, vehicle.y), None)

    def update_vehicle_network(self):
        """
        Deletes the rtree and re initializes it using the self.vehicle_generator() method using bulk upload. This method of bulk upload of all the vehicles at the end of a time interval is preferred as compared to updating it after each vehicle's update.

        """

        # Used bulk upload of data into rtree. Much more faster than individual upload
        del self.__vehicles_tree

        if self.__vehicles_count > 0:
            self.__vehicles_tree = index.Index(self.vehicle_generator())
            # assert len(self.__vehicles_table)==self.__vehicles_count
            # for car in self.get_vehicles_by_bounding_box(bounding_box=self.get_vehicle_tree_bbox(), objects=False):
            #     temp = self.__vehicles_table[car]


def build_sample_network():
    """
    Builds a simple grid like network with + shaped intersections.

    Returns:
        tuple: graph object, intersections table

    """

    graph = nx.DiGraph(name='grid')
    x = itertools.count(1)
    intersections_table = dict()
    start_offset = 30
    road_offsets = 100
    num_int = 16
    for i in range(1,num_int+1):
        # if i%4 in [2,3]:
        #     flag = True
        # else:
        #     flag = True
        #
        ind = choice([0,1,2,3])
        if ind in [1, 2]:
            flag = True
        else:
            flag = True

        rand = choice([1])

        intersections_table[i+1], x = create_plus_intersection(start_offset+rand*road_offsets*((i-1)%4),
                                                               start_offset+road_offsets*((i-1)//4),
                                                               counter=x, scale=map_c.LANE_WIDTH* map_c.NO_OF_LANES,
                                                               traffic_controller=flag)
    

    # i1, x = create_plus_intersection(10, 0+start_offset+0*road_offsets, counter=x, scale=12)
    # intersections_table[1] = i1
    # i2, x = create_plus_intersection(200, 0+offset, counter=x, scale=12)
    # intersections_table[2] = i2
    # i3, x = create_plus_intersection(300, 0+offset, counter=x, scale=12)
    # intersections_table[3] = i3
    # i4, x = create_plus_intersection(400, 0+offset, counter=x, scale=12)
    # intersections_table[4] = i4
    # i5, x = create_plus_intersection(100, 110, counter=x, scale=12)
    # intersections_table[5] = i5
    # i6, x = create_plus_intersection(200, 110, counter=x, scale=12)
    # intersections_table[6] = i6
    # i7, x = create_plus_intersection(300, 110, counter=x, scale=12)
    # intersections_table[7] = i7
    # i8, x = create_plus_intersection(400, 110, counter=x, scale=12)
    # intersections_table[8] = i8
    # i9, x = create_plus_intersection(100, 210, counter=x, scale=12)
    # intersections_table[9] = i9
    # i10, x = create_plus_intersection(200, 300, counter=x, scale=12)
    # intersections_table[10] = i10
    # i11, x = create_plus_intersection(300, 300, counter=x, scale=12)
    # intersections_table[11] = i11
    # i12, x = create_plus_intersection(400, 300, counter=x, scale=12)
    # intersections_table[12] = i12
    # i13, x = create_plus_intersection(100, 400, counter=x, scale=12)
    # intersections_table[13] = i13
    # i14, x = create_plus_intersection(200, 400, counter=x, scale=12)
    # intersections_table[14] = i14
    # i15, x = create_plus_intersection(300, 400, counter=x, scale=12)
    # intersections_table[15] = i15
    # i16, x = create_plus_intersection(400, 400, counter=x, scale=12)
    # intersections_table[16] = i16

    graph = add_intersections_to_graph(graph, intersections_table)

    links = [[6, 9], [10, 5], [14, 17], [18, 13], [22, 25], [26, 21],
             [38, 41], [42, 37], [46, 49], [50, 45], [54, 57], [58, 53],
             [70, 73], [74, 69], [78, 81], [82, 77], [86, 89], [90, 85],
             [102, 105], [106, 101], [110, 113], [114, 109], [118, 121], [122, 117],
             [8, 35], [36, 7], [16, 43], [44, 15], [24, 51], [52, 23], [32, 59], [60, 31],
             [40, 67], [68, 39], [48, 75], [76, 47], [56, 83], [84, 55], [64, 91], [92, 63],
             [72, 99], [100, 71], [80, 107], [108, 79], [88, 115], [116, 87], [96, 123], [124, 95]
             ]

    for link in links:
        temp_node1 = graph.node[link[0]]['object']
        temp_node2 = graph.node[link[1]]['object']

        ls = LineString([(temp_node1.x, temp_node1.y), (temp_node2.x, temp_node2.y)])
        # rs = RoadString(ls)
        temp_road = Road(start_node=temp_node1, end_node=temp_node2, line_string=ls)
        graph.add_edges_from([(temp_node1.get_id(), temp_node2.get_id(),
                               {'object': temp_road, 'distance': temp_road.road_length,
                                'time': temp_road.avg_time, 'traffic': temp_road.val})])

    for road in graph.edges(data=True):
        # print(type(road[2]), road)
        traffic_signals = road[2]['object'].end_node.traffic_signals
        # if len(traffic_signals) > 0:
        num_lanes = road[2]['object'].no_of_lanes
        for i in range(num_lanes):
            lane = road[2]['object'].lanes[i]
            x, y = lane.lane_center_xy[2], lane.lane_center_xy[3]
            if traffic_signals[i] is not None:
                traffic_signals[i].set_coords(x, y)

    return graph, intersections_table


def add_intersections_to_graph(graph, intersections_table):
    """
    Iterates over the intersections and adds nodes to the graph, and create edges (Roads) from nodes that are connected. This piece of code assumes the simple plus shaped intersections, and the turn_links variable is based on this assumption.

    Notes:
        Future implementations must consider various shapes of intersections and different combinations of turn roads
        to be added to the intersection.

    Args:
        graph (object): Networkx graph object
        intersections_table (dict): dict containing the intersection objects

    Returns:
        object: Graph object

    """

    for _, intersection in intersections_table.items():
        for node in intersection.get_nodes():
            graph.add_node(node.get_id(), object=node)

        turn_links = [[0, 3], [0, 5], [0, 7], [2, 5], [2, 7], [2, 1], [4, 7], [4, 1], [4, 3], [6, 1], [6, 3], [6, 5]]

        for link in turn_links:
            temp_node1 = intersection.get_node_by_id(link[0])
            temp_node2 = intersection.get_node_by_id(link[1])

            ls = LineString([(temp_node1.x, temp_node1.y), (temp_node2.x, temp_node2.y)])
            # rs = RoadString(ls)
            temp_road = Road(start_node=temp_node1, end_node=temp_node2, line_string=ls, _type='turn')
            graph.add_edges_from([(temp_node1.get_id(), temp_node2.get_id(),
                                   {'object': temp_road, 'distance': temp_road.road_length,
                                    'time': temp_road.avg_time, 'traffic': temp_road.val})])

        nodes = intersection.get_nodes()
        traffic_controller = intersection.traffic_controller
        for i in range(len(nodes)):
            if i in [0, 2, 4, 6]:
                if traffic_controller is not None:
                    nodes[i].traffic_signals[0] = traffic_controller.signals[int(3 * i/2)]
                    traffic_controller.signals[int(3*i/2)].set_node(nodes[i])
                    nodes[i].traffic_signals[1] = traffic_controller.signals[int(3 * i/2)+1]
                    traffic_controller.signals[int(3*i/2)+1].set_node(nodes[i])
                    nodes[i].traffic_signals[2] = traffic_controller.signals[int(3 * i/2)+2]
                    traffic_controller.signals[int(3*i/2)+2].set_node(nodes[i])

    return graph


def create_plus_intersection(point_x, point_y, counter, scale=12, traffic_controller=True):
    """
    Creates a + shaped intersection and instantiates nodes (in a fixed fashion).

    Notes:
        Future implementations must include methods to create arbitrary (shaped/sized) intersections.

    Args:
        point_x (float): x coordinate
        point_y (float): y coordinate
        counter (int): used to number the nodes (node ids)
        scale (int): scaling factor of the coordinates (since the nodes are placed here wrt distance from intersection)
        traffic_controller (object): traffic controller on the intersection

    Returns:
        tuple: Intersection object, and counter number (used to sync the node ids on the graph)

    """

    if not traffic_controller:
        traffic_controller = None
    else:
        traffic_controller = TrafficController()

    intersection = Intersection(point_x, point_y, traffic_controller=traffic_controller)

    intersection.add_node(Node(point_x-1*scale, point_y+0.5*scale, incoming=True,
                               _id=next(counter), parent=intersection))
    intersection.add_node(Node(point_x-1*scale, point_y-0.5*scale, incoming=False,
                               _id=next(counter), parent=intersection))
    intersection.add_node(Node(point_x-0.5*scale, point_y-1*scale, incoming=True,
                               _id=next(counter), parent=intersection))
    intersection.add_node(Node(point_x+0.5*scale, point_y-1*scale, incoming=False,
                               _id=next(counter), parent=intersection))
    intersection.add_node(Node(point_x+1*scale, point_y-0.5*scale, incoming=True,
                               _id=next(counter), parent=intersection))
    intersection.add_node(Node(point_x+1*scale, point_y+0.5*scale, incoming=False,
                               _id=next(counter), parent=intersection))
    intersection.add_node(Node(point_x+0.5*scale, point_y+1*scale, incoming=True,
                               _id=next(counter), parent=intersection))
    intersection.add_node(Node(point_x-0.5*scale, point_y+1*scale, incoming=False,
                               _id=next(counter), parent=intersection))

    return intersection, counter


def polygon_from_linestring(linestring, width_from_center):
    """
    Generates a polygon object around the given line string with the given width from center.

    Args:
        linestring (object) : Shapely LineString object
        width_from_center (float) : distance from the center to either sides (Edges) of the polygon

    Returns:
        object: Shapely Polygon object

    """

    pts = []
    for x, y in zip(linestring.coords.xy[0], linestring.coords.xy[1]):
        pts.append([x, y])

    theta = math.atan2((pts[1][1] - pts[0][1]), (pts[1][0] - pts[0][0]))
    w = width_from_center
    pt1 = (pts[0][0] - w * math.sin(theta), pts[0][1] + w * math.cos(theta))
    pt2 = (pts[0][0] + w * math.sin(theta), pts[0][1] - w * math.cos(theta))

    theta = math.atan2(-(pts[1][1] - pts[0][1]), -(pts[1][0] - pts[0][0]))
    pt3 = (pts[1][0] - w * math.sin(theta), pts[1][1] + w * math.cos(theta))
    pt4 = (pts[1][0] + w * math.sin(theta), pts[1][1] - w * math.cos(theta))

    polygon = Polygon([pt1, pt2, pt3, pt4])

    return polygon
