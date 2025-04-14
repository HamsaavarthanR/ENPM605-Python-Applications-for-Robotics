# Camera class (Subclass of Sensor)

# Import modules
from models.sensor.sensor_interface import Sensor


class Camera(Sensor):
    """
    Class that represents a Camera senor for imaging.

    Attributes:
        _pixel_resolution (int): Camera resolution in megapixels (MP)
        
    Args:
        Sensor (class): Parent Sensor class with (model, range) informations
    """
    # Constructor
    def __init__(self, model: str, range: float, pixel_resolution: int):
        """
        Camera class constructor to initialise Camera class attributes

        Args:
            model (str): Camera Sensor model
            range (float): Camera Sensor range
            pixel_resolution (int): Camera Sensor resolution in megapixels (MP)
        """
        # Call Parent class constructor
        super().__init__(model, range)
        
        # Attributes (non-public)
        self._pixel_resolution = pixel_resolution
        
    # Encapsulation
    @property
    def pixel_resolution(self):
        return self._pixel_resolution
        
    # Methods (Override)
    def get_info(self):
        """
        Function overriden from 'Sensor' class to provide camera specific informations.

        Returns:
            info (str): camera information with (model, range, and pixel resolution)
        """
        # Camera Information
        info = f"Camera (model: {self._model}, range: {self._range}m, pixel resolution: {self._pixel_resolution}MP)"
        return info
    
    def operate(self):
        """
        Function overriden from 'Sensor' class to provide camera specific operations.

        Returns:
            operation (str): Camera specific operation.
        """
        
        # Camera Operation
        operation = f"{self._model} (Camera): Scanning terrain with {self._pixel_resolution}MP resolution"
        return operation