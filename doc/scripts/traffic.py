"""
Traffic module contains classes and methods for controlling the traffic signals in the simulator.

"""
from core.lib.infrastructure.utils import constants as tfc

# Todo: Add dynamic assignment of traffic controller (and signals) to an arbitrarily created intersection.
# Todo: Add config file for TrafficController to enable ease of use.


class TrafficController:

    """Controls and synchronises all the traffic signals at an intersection

    The number of signals on a controller and the number of phases are coupled. In the current implementation, a fixed number of traffic signals is assumed (12) and the no of phases is fixed. The future implementation must include dynamically changing the "__no_phases" if a controller is instantiated with a different number of signals.

    Attributes:
        __no_phases (int): indicates the number of phases (this depends on the functionality and no of signals)
        __dt (float): time between successive traffic state updates
        __scale(float):
        __cycle_length (float): cycle length is the time taken for one complete cycle (a signal going from g-y-r-g)
        __table (dict): table defines the times at which the signals should switch. The entire control lies with this
                        table
        signals(list): list of TrafficSignal objects

    """

    def __init__(self):

        """
        Initializes the controller with the no of phases, the frequency of update, cycle length. Includes a public attribute "signals" that is a dict of TrafficSignal objects.
        """
        # self.signals = signals  # signals is a dict with keys in {0,1,..,7}
        self.__no_phases = tfc.NO_OF_PHASES
        self.__dt = tfc.TRAFFIC_CONTROLLER_DT
        self.__scale = self.__dt/0.1
        self.__cycle_length = tfc.TRAFFIC_SIGNAL_CYCLE * self.__scale
        self.__table = self.generate_table()
        self.signals = dict.fromkeys(list(range(8)))
        self.__set_signals()

    def __set_signals(self):

        self.signals = dict.fromkeys(range(12))
        for i in range(12):
            self.signals[i] = TrafficSignal(tcid=i)

    def set_table(self, table):
        """
        Currently a public method to enable the users to set a custom and phase timing (relative to the start of the cycle).

        Args:
            table: A dict with keys as the indices to the traffic signals and values as a list containing the phase

        :Example:

        Consider a TrafficController of cycle length 30. The entry for signal 0 in the self.__table looks like this.

        ::

            table[0] = ['g', 10, 'y', 15, 'r', 30]

        Traffic Signal at index 0 is green for the first 10 seconds, yellow for the next five seconds, and red for the next 15 seconds. The last entry should match with the cycle length of the TrafficController.

        """
        self.__table = table

    def generate_table(self):

        """
        Generates a table as described above with specified green time ratio, yellow time ratio and red time ratio. These ratios are coupled with the phase of the signal so care must be taken while changing these values.

        Returns:
            dict: keys as the traffic signal tcid, and the value as a list described in the above example
        """

        table = {}
        gr = 0.25
        ar = 0.05
        rr = 0.75
        gt = self.__cycle_length* gr   #green time
        at = self.__cycle_length* ar  #amber time
        rt = self.__cycle_length - gt - at

        # gt = 4* self.__scale   #green time
        # at = 1* self.__scale  #amber time
        # rt = self.__cycle_length - gt - at

        table[0] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        table[1] = ['r', gt+at, 'g', 2*gt + at, 'a', 2*(gt+at), 'r', self.__cycle_length]
        table[6] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        table[7] = ['r', gt+at, 'g', 2*gt + at, 'a', 2*(gt+at), 'r', self.__cycle_length]
        table[3] = ['r', 2*(gt+at), 'g', 2*(gt+at)+gt, 'a', 3*(gt+at), 'r', self.__cycle_length]
        table[4] = ['r', 3*(gt+at), 'g', 3*(gt+at)+gt, 'a', self.__cycle_length]
        table[9] = ['r', 2*(gt+at), 'g', 2*(gt+at)+gt, 'a', 3*(gt+at), 'r', self.__cycle_length]
        table[10] = ['r', 3*(gt+at), 'g', 3*(gt+at)+gt, 'a', self.__cycle_length]

        # table[1] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[7] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        #
        # table[0] = ['r', gt+at, 'g', gt+at+gt, 'r', self.__cycle_length]
        # table[6] = ['r', gt+at, 'g', gt+at+gt, 'r', self.__cycle_length]
        #
        # table[4] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]
        # table[9] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]
        # table[10] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]

        # ToDo: add constants for 'g','r','y'
        # table[0] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[1] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[6] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[7] = ['g', gt, 'a', gt+at, 'r', self.__cycle_length]
        # table[3] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]
        # table[4] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]
        # table[9] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]
        # table[10] = ['r', gt, 'g', gt+rt, 'a', self.__cycle_length]

        table[2] = ['g', self.__cycle_length, 'a', self.__cycle_length, 'r', self.__cycle_length]
        table[5] = ['g', self.__cycle_length, 'a', self.__cycle_length, 'r', self.__cycle_length]
        table[8] = ['g', self.__cycle_length, 'a', self.__cycle_length, 'r', self.__cycle_length]
        table[11] = ['g', self.__cycle_length, 'a', self.__cycle_length, 'r', self.__cycle_length]

        return table

    def print_table(self):
        for i in self.__table.keys():
            print(i, self.__table[i])

    def update_signals(self):

        """
        This method is called every update step in the main simulation loop to update all the signal states and timings.

        """

        for signal in self.signals.values():
            signal.tick += 1
            # reset cycle
            # print(self.__dt, self.__cycle_length)
            if signal.tick * self.__dt > self.__cycle_length:
                signal.tick = 0

            for c_ind in range(0, len(self.__table[signal.tcid]), 2):
                # print(self.__table[signal.tcid][c_ind+1])
                if signal.tick * self.__dt < self.__table[signal.tcid][c_ind+1]:
                    color = self.__table[signal.tcid][c_ind]
                    time_remaining = self.__table[signal.tcid][c_ind+1] - signal.tick * self.__dt
                    signal.set_state(color, time_remaining)
                    break

    def reset_signals(self):
        """
        Resets all the signals to the beginning of the time cycle. This can be used to adaptively change the relative phase of the all the signals.

        """
        for signal in self.signals.values():
            signal.tick = 0
            signal.color = None
            signal.time_remaining = None


class TrafficSignal:

    """Traffic Signal is an interface for the vehicle to 'see' the states. It is controlled by a TrafficController.

    Attributes:
        color (str): indicates the traffic state (red, green, yellow/amber)
        time_remaining (float): time after which the color changes
        tcid (int): unique identifier of the signal wrt to the controller
        node (Node): the node that has the traffic signal
        tick (int): counter/ time
        x (object): x coord of signal
        y (object): y coord of signal

    """

    def __init__(self, tcid):
        """
        Constructor for class TrafficSignal.

        Args:
            tcid (int): identifier to distinguish the signal from the other signals at the same intersection.

        """
        self.color = None
        self.time_remaining = None
        self.tcid = tcid
        self.node = None
        self.tick = 0
        self.x = None
        self.y = None

    def set_node(self, node):
        """
        Links the signal to a node object, to enable referencing nodes while accessing signals. It is used while creating a map and assigning traffic signals to end nodes of roads ending at intersections.

        Args:
            node (Node): the node object to which the signal is assigned

        """
        self.node = node

    def set_coords(self, x, y):
        """
        Sets the coordinates of the signal.

        Args:
            x (object): x coord of signal
            y (object): y coord of signal

        """
        self.x = x
        self.y = y

    def set_state(self, color, time_remaining):
        """
        Sets the state of the signals regarding to the color and remaining time.

        Args:
            color (str): 'r' for red, 'g' for green, 'a' for amber/yellow
            time_remaining (float): time remaining in the current phase (time after which the color changes)

        """
        self.color = color
        self.time_remaining = time_remaining

    def get_state(self):

        """
        Enables direct querying of the traffic signals' states by the vehicles or any other object that has access to nodes. In the future implementation, communication module (v2x) should be interfaced with  traffic signals.

        Returns:
              list: [color, time remaining]

        """
        return [self.color, self.time_remaining]
