# SpaceRobot class (Abstract Base Class)

# Import modules
from abc import ABC, abstractmethod

# from sensor import sensorsystem_interface
from models.sensor.sensorsystem_interface import SensorSyestem


class SpaceRobot(ABC):
    """
    Abstract class that defines the common structure and behaviour for all space robot types.

    """
    # Constructor
    def __init__(self, name: str, sensor_system: SensorSyestem, mobility: str):
        """
        SpaceRobot class constructor to initialise class attributes.

        Args:
            name (str): Name of the space robot.
            sensor_system (sensorsystem_interface.SensorSyestem): System of sensors from 'SensorSystem' class.
            mobility (str): mobility mechanism the space robot uses to perform tasks.
        """
        # Attributes (non-public)
        self._name = name
        self._sensor_system = sensor_system
        self._mobility = mobility
        
    # Encapsulation
    @property
    def name(self):
        return self._name
    
    @property
    def sensor_system(self):
        return self._sensor_system
    
    @property
    def mobility(self):
        return self._mobility
        
    # Methods (Abstract)
    @abstractmethod
    def perform_task(self, task) -> str:
        """
        Abstract method that subclasses must override to perform a task by the robot.

        Args:
            task (str): Task that must be performed by the robot.
        """
        pass
    
    # Method (Instance)
    def activate(self):
        """
        Sets the sensor system's current status to "active".

        Returns:
            infos (str): String listing all sensors in the system.
        """
        return self._sensor_system.activate()
        
