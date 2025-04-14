# OrbitacRobot class (Subclass of SpaceRobot)

# Import modules
from models.robot.spacerobot_interface import SpaceRobot

import sys
sys.path.append('../sensor')
from models.sensor.sensorsystem_interface import SensorSyestem


class OrbitalRobot(SpaceRobot):
    """
    Class that represents a robotic system for serving space stations and satellites in orbit.

    Args:
        SpaceRobot (class): Parent class to represent Space Robot.
        
    """
    # Constructor
    def __init__(self, name: str, sensor_system: SensorSyestem, mobility: str, orbit_altitude: float):
        """
        OrbitalRobot class constructor to initialies class attributes. 
        Args:
            name (str): Name of the space robot.
            sensor_system (sensorsystem_interface.SensorSyestem): System of sensors from 'SensorSystem' class.
            mobility (str): mobility mechanism the space robot uses to perform tasks.
            orbital_altitude (float): Altitude at which the orbital robot performs tasks.
        """
        # Call parent class constructor
        super().__init__(name, sensor_system, mobility)
        # Attributes (non-public)
        self._orbit_altitude = orbit_altitude
        
        
    # Encapsulation
    @property
    def orbit_altitude(self):
        return self._orbit_altitude
        
    # Methods (Ovveride)
    def perform_task(self, task):
        """
        Orbital robot performs one of the (repair/ maintenance) tasks.

        Args:
            task (str): Task information that the orbital robot must perform.

        Returns:
            str: Returns a string of the performed task.
                 Returns None if tha task cannot be performed.
        """
        # Task: repair
        if task == "repair":
            # Displays the robot's model
            print(f"- Orbital Robot: '{self._name}' (perfomring: {task}")
            # Calls move
            print(f"        {self.move()}")
            print("        Repairing: Satellite")
            
        # Task: "maintenance"
        elif task == "maintenance":
            # Displays the robot's model
            print(f"- Orbital Robot: '{self._name}' (perfomring: {task}")
            # Calls move
            print(f"        {self.move()}")
            print("        Inspecting: Station hull")  
        
        # Task: Others
        elif task not in ["repain", "maintenance"]:
            print(f"Error: {self._name} orbital robot cannot perform {task} task!")
            return
        
        # Returns a string of performed task.
        return task
    
    # Methods (Instance)
    def move(self):
        """
        Performs orbiting action of the orbital robot over the space at given altitude.

        Returns:
            str: Returns a string information on the movement of the orbital robot.
        """
        return f"Orbiting: Using {self._mobility} at {self._orbit_altitude} km altitude"
        
        