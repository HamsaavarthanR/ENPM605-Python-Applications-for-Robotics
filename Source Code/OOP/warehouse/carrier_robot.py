from warehouse.robot import Robot


class CarrierRobot(Robot):
    def __init__(
        self, robot_id: str, model: str, battery_capacity: int, load_capacity: float
    ):
        if load_capacity <= 0:
            raise ValueError("Load capacity must be a positive integer.")

        super().__init__(robot_id, model, battery_capacity)  # Call Robot __init__
        self._load_capacity = load_capacity

    @property
    def load_capacity(self):
        """
        Specifies the maximum weight the robot can carry.
        """
        return self._load_capacity

    def perform_task(self):
        print(f"{self._robot_id} is transporting materials")
