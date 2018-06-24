import random
from scipy.stats import skewnorm


class Driver:
    """Driver object is a collection of attributes that define the behavior of a vehicle (with a human at the steering wheel).

    Driver has two private attributes

        * safety affects the safe distance that the driver wants to maintain with the car in front.
        * aggression affects the probability of speeding over the specified road speed limit.

    Attributes:
        __safety (float): between 0 and 1. Drawn from a skew distribution, determined by config
        __aggression (float): between 0 and 1. Drawn from a skew distribution, determined by config
    """

    def __init__(self, config):

        self.__aggression = config.aggression or random.random()
        self.__safety = config.safety or random.random()

        if self.__safety > 0.5:
            self.__safety = skewnorm.rvs(1, size=1)[0]
        else:
            self.__safety = skewnorm.rvs(-1, size=1)[0]

        if self.__aggression > 0.5:
            self.__aggression = skewnorm.rvs(1, size=1)[0]
        else:
            self.__aggression = skewnorm.rvs(-1, size=1)[0]

    @property
    def aggression(self):
        return self.__aggression

    @property
    def safety(self):
        return self.__safety


class IntelligentDriver(Driver):
    """Inherits Driver class.

    An utopian driver following the rules and regulations perfectly.
    
    Attributes:
        Driver (Driver): Base driver class

    """

    def __init__(self, config):
        if not config.aggression == 0:
            config.aggression = 0
        if not config.safety == 1:
            config.safety = 1
        super().__init__(config)



