from warehouse.robot import Robot


class SorterRobot(Robot):
    def __init__(
        self, robot_id: str, model: str, battery_capacity: int, sorting_speed: int
    ):
        if sorting_speed <= 0:
            raise ValueError("Sorting speed must be a positive integer.")
        super().__init__(robot_id, model, battery_capacity)  # Call Robot __init__
        self._sorting_speed = sorting_speed

    @property
    def sorting_speed(self):
        """
        Determines the number of packages the robot can sort per minute.
        """
        return self._sorting_speed
    
    def perform_task(self):
        print(f"{self._robot_id} is sorting packages.")
