from sensor import GPS, LaneCenterSensor, PositionBasedVelocityEstimator, BasicVisual


class SensorFactory:
    """Factory class to build sensors of various type.

    """
    @staticmethod
    def build_sensor(type):
        """
        
        Args:
            type (str): should be a valid sensor type among the following implemented

        Returns:
            object: returns an instantiation of the mentioned sensor

        Notes:
            Currently only includes the following sensors.
                - 'gps'
                - 'lane_center'
                - 'speed'
                - 'basic_visual'

        """
        if type == 'gps':
            return GPS()
        elif type == 'lane_center':
            return LaneCenterSensor()
        elif type == 'speed':
            return PositionBasedVelocityEstimator()
        elif type == 'basic_visual':
            return BasicVisual()

