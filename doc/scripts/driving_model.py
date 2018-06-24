from core.lib.adas.lane_center import LaneCenter
from core.lib.adas.lane_change import LaneChange
from core.lib.adas.longitudinal_contrl import LongitudinalControl
from core.lib.adas.rules import Rules


class ADAS:
    """Provides driver assistance functions

    Attributes:
        driver (object): object that maintains driver attributes
        longitudinal_control (object): object that controls longitudinal dynamics
        lane_change (object): object that monitors lane_change control
        lane_center (object): object that controls lane_centering

    """
    def __init__(self, driver):
        """

        Args:
            driver (object): driver maintains the attributes like aggression or safety. This object is used in other
            ADAS attributes for several controls.
        """
        self.driver = driver
        self.longitudinal_control = LongitudinalControl(driver)
        self.lane_change = LaneChange()
        self.lane_center = LaneCenter()
        # self.rules = Rules()
