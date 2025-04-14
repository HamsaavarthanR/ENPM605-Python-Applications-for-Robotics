# LiDAR Class (Subclass of Sensor)

# Import Modules
from models.sensor.sensor_interface import Sensor

# UniCode Character
degree_sign = u'\N{DEGREE SIGN}'


class LiDAR(Sensor):
    """
    Class that represents a LiDAR senor for distance measurement and mapping.

    Attributes:
        _angular_resolution (float): Angular resolution in degrees, affecting scan precision.
        
    Args:
        Sensor (class): Parent Sensor class with (model, range) informations
    """
    # Constructor
    def __init__(self, model: str, range: float, angular_resolution: float):
        """
        LiDAR class constructor to initialise LiDAR class attributes

        Args:
            model (str): LiDAR Sensor model
            range (float): LiDAR Sensor range
            angular_resolution (float): LiDAR Sensor angular resolution in degrees
        """
        # Call Parent Class Constructor
        super().__init__(model, range)
        
        # Attributes (non-public)
        self._angular_resolution = angular_resolution
        
    # Encapsulation
    @property
    def angular_resolution(self):
        return self._angular_resolution
        
    # Methods (Override)
    def get_info(self):
        """
        Function overriden from 'Sensor' class to provide LiDAR specific informations.

        Returns:
            info (str): LiDAR information with (model, range, and angular resolution)
        """
        # LiDAR Information
        info = f"LiDAR (model: {self._model}, range: {self._range}m, angular resolution: {self._angular_resolution}{degree_sign})"
        return info
    
    def operate(self):
        """
        Function overriden from 'Sensor' class to provide LiDAR specific operations.

        Returns:
            operation (str): LiDAR specific operation.
        """
        
        # LiDAR Operation
        operation = f"{self._model} (LiDAR): Scanning terrain with {self._angular_resolution}{degree_sign} resolution"
        return operation
         