# Sensor Class (Abstract Base Class)

# Import modules
from abc import ABC, abstractmethod

class Sensor(ABC):
    """
    Class that defines a common structure and behavior for all sensor types, providing abstraction.
    """
    # Constructor
    def __init__(self, model: str, range: float):
        """
        Sensor class constructor to initialise class attributes.

        Args:
            model (str): Sensor model
            range (float): Sensor range
        """
        # Attributes (non-public)
        self._model = model
        self._range = range
        
    # Encapsulation
    @property
    def model(self):
        return self._model
    
    @property
    def range(self):
        return self._range
        
    # Methods (Abstract)
    @abstractmethod
    def get_info(self) -> str:
        """
        Absrtact function that the subclasses must override.

        Returns:
            str: Describes the sensor.
        """
        pass
    
    @abstractmethod
    def operate(self) -> str:
        """
        Absrtact function that the subclasses must override.

        Returns:
            str: Describes sensor's specific oprations.
        """
        pass