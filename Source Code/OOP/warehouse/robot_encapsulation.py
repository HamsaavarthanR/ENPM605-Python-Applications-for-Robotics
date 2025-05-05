class Robot(object):
    def __init__(self, robot_id: str, model: str, battery_charge_level: float) -> None:
        self.robot_id = robot_id
        self.model = model
        self.battery_charge_level = battery_charge_level

    def move(self, destination: str, time_required) -> None:
        if self.battery_charge_level >= time_required:
            print(f"{self.model} Robot {self.robot_id} is moving to {destination}")
        else:
            print(
                f"{self.model} Robot {self.robot_id} has insufficient battery to move!"
            )

    def __str__(self):
        return (
            f"Robot ID: {self.robot_id}\n"
            f"Model: {self.model}\n"
            f"Battery Charge Level: {self.battery_charge_level:.2f}"
        )

    def __call__(self, task: str) -> None:
        """Allows the robot to be 'called' like a function to perform a task."""
        print(f"Robot {self.robot_id} is executing task: {task}")

    def __gt__(self, other):
        if isinstance(other, Robot):
            return self.battery_charge_level > other.battery_charge_level
        else:
            raise TypeError("Unsupported operand types for >")
        
    # def get_robot_id(self):
    #     return self._robot_id

    # def get_model(self):
    #     return self._model
    
    # def get_battery_charge_level(self):
    #     return self._battery_charge_level

    # def set_robot_id(self, robot_id):
    #     if not isinstance(robot_id, str):
    #         raise TypeError("robot_id must be a string.")
    #     self._robot_id = robot_id
    
    # def del_robot_id(self):
    #     if hasattr(self, "_robot_id"):
    #         print("Deleting robot_id...")
    #         del self._robot_id  # Removes the attribute from the instance
    #     else:
    #         print("robot_id does not exist.")
        
    # robot_id = property(fget=_get_robot_id, fset=_set_robot_id, doc="ID of the robot")
    # robot_id = property(fget=_get_robot_id, fset=_set_robot_id, fdel=_del_robot_id, doc="ID of the robot")
    