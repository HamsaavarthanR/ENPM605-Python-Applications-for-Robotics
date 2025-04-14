# PlanetaryRover class (Subclass of SpaceRobot)

# Import modules
from models.robot.spacerobot_interface import SpaceRobot

import sys
sys.path.append('../sensor')
from models.sensor.sensorsystem_interface import SensorSyestem


class PlanetaryRobot(SpaceRobot):
    """
    Class that represents a robotics rover designed for planetary surface explorations.

    Args:
        SpaceRobot (class): Parent class to represent Space Robot.
        
    """
    # Constructor
    def __init__(self, name: str, sensor_system: SensorSyestem, mobility: str, terrain_type: str):
        """
        PlanetaryRobot class constructor to initialies class attributes. 
        Args:
            name (str): Name of the space robot.
            sensor_system (sensorsystem_interface.SensorSyestem): System of sensors from 'SensorSystem' class.
            mobility (str): mobility mechanism the space robot uses to perform tasks.
            terrain_type (str): Type of terrain that the robot traverse 
        """
        # Call Parent class constructor
        super().__init__(name, sensor_system, mobility)
        
        # Attributes (non-public)
        self._terrain_type = terrain_type
        
    # Encapsulation
    @property
    def terrain_type(self):
        return self._terrain_type
        
    # Methods (Override)
    def perform_task(self, task: str) -> str:
        """
        Planetary robot performs one of the (mapping/ collection) tasks.

        Args:
            task (str): Task information that the planetary robot must perform.

        Returns:
            str: Returns a string of the performed task.
                 Returns None if tha task cannot be performed.
        """
        
        # Task: "mapping"
        if task == "mapping":
            # Displays robot's model
            print(f"- Space Robot: '{self._name}' (perfomring: {task} over {self._terrain_type} terrain)")
            
        # Task: "collection"
        elif task == "collection":
            # Displays robot's modelttttttt
            print(f"- Space Robot: '{self._name}' (perfomring: {task} over {self._terrain_type} terrain)")
            # Calls move
            self.move()
            print(f"        Moving: COllecting smaple on {self._terrain_type} terrain")
            # Calls operate_sensors() on the _sensor_system
            operations_list = self._sensor_system.operate_sensors().split(",")
            for ops in operations_list:
                print(f"        {ops}")
            
        # Task: Others
        elif task not in ["mapping", "collection"]:
            print(f"Error: {self._name} planetary robot cannot perform {task} task!")
            return
            
        # Return string of performed task
        return task
            
        
    
    # Methods (Instance)
    def move(self):
        """
        Performs moving action of the planetary robot over the respective terrain.

        Returns:
            str: Returns a string information on the movement of the planetary robot.
        """
        return f"Moving: Using {self._mobility} across {self._terrain_type} terrain"