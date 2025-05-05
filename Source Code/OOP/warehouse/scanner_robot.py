from warehouse.robot import Robot


class ScannerRobot(Robot):
    def __init__(
        self, robot_id: str, model: str, battery_capacity: int, scan_range: float
    ):
        if scan_range <= 0:
            raise ValueError("Scan range must be a positive integer.")
        super().__init__(robot_id, model, battery_capacity)  # Call Robot __init__
        self._scan_range = scan_range

    @property
    def scan_range(self):
        """
        Defines the maximum scanning distance.
        """
        return self._scan_range
    
    def perform_task(self):
        print(f"{self._robot_id} is scanning bar codes.")
