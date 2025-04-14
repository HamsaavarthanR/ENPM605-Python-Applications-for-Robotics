# SensorSystem class

# Import modules
from models.sensor.sensor_interface import Sensor
from typing import List

class SensorSyestem():
    """
    Class that manages Sensor subclass instances.

    """
    # Constructor
    def __init__(self, sensors: List[Sensor]):
        """
        SensorSystem class constructor to initialise class attributes.
        
        Attributes:
            _status (str): Current status (active/ inactive) of the system. Initially set to "inactive".

        Args:
            sensors (List[Sensor]): List of sensor objects.
        """
        # Attributes (non-public)
        self._sensors = sensors
        self._status = "inactive" # By-default set to "inactive" when SensorSytem object is initialised
        
    # Encapsulation
    @property
    def sensors(self):
        return self._sensors
    
    @property
    def status(self):
        return self._status
    
    # Methods
    def activate(self):
        """
        Sets the system's current status to "active"._

        Returns:
            infos (str): String listing all sensors in the system.
        """
        # Set status to active
        self._status = "active"
        # Create empty string to store sensor informations
        infos = ''
        for sensor in self._sensors:
            infos += f"{sensor.get_info()},"
        return infos
    
    def operate_sensors(self):
        """
        Operates all sensors in the system.  

        Returns:
            ops (str): String combining all sensor's operate() outputs.
        """
        # Create empty string to store operate outputs
        ops = ''
        for sensor in self._sensors:
            ops += f"{sensor.operate()},"
        return ops
            
    
        