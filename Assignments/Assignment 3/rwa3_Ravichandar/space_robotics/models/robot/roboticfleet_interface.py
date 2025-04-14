# RoboticFleet class

# Import modules
from models.robot.spacerobot_interface import SpaceRobot
from typing import List, Optional


class RoboticsFleet():
    # Constructor
    def __init__(self, robots: Optional[List[SpaceRobot]]):
        # Attributes (non-public)
        if robots is None:
            self._robots = []
        else:
            self._robots = robots
            
    # Attribute Encapsulation
    def robot_get(self):
        return self._robots
    
    # Methods (Instance)
    def add_robot(self, robot: SpaceRobot) -> str:
        self._robots.append(robot)
        return f"Added {robot._name} to fleet"
    
    def remove_robot(self, robot: SpaceRobot) -> str:
        if robot in self._robots:
            self._robots.remove(robot)
            return f"Removed {robot._name} from fleet"
        else:
            print("Robot not in fleet!")
            
    def deploy_mission(self, task: str) -> str:
        if self._robots != []:
            # Create empty tring to store tasks performed
            tasks = ''
            for robot in self._robots:
                tasks += f"{robot._name}: {robot.perform_task(task)},"
        else:
            print("Fleet is empty! Add robots to deploy mission.")
        return tasks
    
    def report_status(self) -> str:
        if self._robots != []:
            # Create empty string to store robot's status
            status = ''
            for robot in self._robots:
                status += f"-Robot: {robot._name}  Mobility: {robot._mobility},"
                
            return status
        else:
            return "Fleet is empty!"