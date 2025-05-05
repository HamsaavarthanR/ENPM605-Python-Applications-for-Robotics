import re
from warehouse.operator import Operator
from warehouse.battery import Battery


class Robot:
    _total_robots = 0  # Class attribute shared by all instances

    def __init__(self, robot_id: str, model: str, battery_capacity: int):
        if Robot._validate_robot_id(robot_id):
            self._robot_id = robot_id
        self._model = model
        self._operator = Operator("N/A")
        self._battery = Battery(battery_capacity)
        Robot._total_robots += 1  # Increment class attribute

    @property
    def operator(self):
        """
        Name of the operator
        """
        return self._operator

    # @operator.setter
    # def operator(self, operator:Operator):
    #     self._operator = operator
    #     print(f"Robot {self._robot_id} has been assigned a new operator: {self._operator.name}")

    @property
    def robot_id(self):
        """
        ID of the robot
        """
        return self._robot_id

    @robot_id.setter
    def robot_id(self, robot_id):
        if not isinstance(robot_id, str):
            raise TypeError("robot_id must be a string.")
        self._robot_id = robot_id

    @property
    def model(self):
        """
        Model of the robot
        """
        return self._model

    @model.setter
    def model(self, model):
        raise AttributeError("Modifying the model is not allowed.")

    @property
    def battery(self):
        """
        Battery attached to the robot
        """
        return self._battery

    @staticmethod
    def _validate_robot_id(robot_id):
        """
        Ensures that the robot_id follows the format:
        - Letters before the dash
        - Numbers after the dash
        - Contains exactly one dash
        Example of a valid format: "TN-8800"
        """
        pattern = (
            r"^[A-Za-z]+-\d+$"  # Regex pattern: Letters before "-", numbers after "-"
        )

        if isinstance(robot_id, str) and re.fullmatch(pattern, robot_id):
            return True
        else:
            raise ValueError(
                "Invalid model format.\n"
                "Expected format: 'Letters-Numbers' (e.g., 'TN-8800')."
            )

    @classmethod
    def get_total_robots(cls):
        """Returns the total number of Robot instances created."""
        return cls._total_robots

    @classmethod
    def build_from_id(cls, robot_id, battery_charge_level):
        """Create a robot object from the robot_id."""
        model_prefix = robot_id.split("-")[0]  # Extract the prefix before '-'

        # Mapping robot ID prefixes to models
        model_mapping = {
            "TN": "Titan",
            "NMD": "Nomad",
            "XR": "Xplorer",
            "ALPHA": "AlphaBot",
            "ZK": "ZenithKnight",
        }

        # Ensure the prefix matches a known model
        if model_prefix not in model_mapping:
            raise ValueError(
                f"Invalid robot ID prefix: '{model_prefix}'.\n"
                f"Expected one of {list(model_mapping.keys())}."
            )

        model = model_mapping[model_prefix]

        return cls(robot_id, model, battery_charge_level)

    def __str__(self):
        return (
            f"Robot ID: {self._robot_id}\n"
            f"Model: {self._model}\n"
            f"Operator: {self._operator.name}\n"
            f"Battery Charge Level: {self._battery.charge_level}"
        )

    def __call__(self, task: str) -> None:
        """Allows the robot to be 'called' like a function to perform a task."""
        print(f"Robot {self._robot_id} is executing task: {task}")

    def __gt__(self, other):
        if isinstance(other, Robot):
            return self._battery.charge_level > other._battery.charge_level
        else:
            raise TypeError("Unsupported operand types for >")

    def move(self, destination: str, time_required) -> None:
        if self._battery.charge_level >= time_required:
            print(f"{self._model} Robot {self._robot_id} is moving to {destination}")
            self._battery.use(time_required)
        else:
            print(
                f"{self._model} Robot {self._robot_id} has insufficient battery to move...recharging"
            )
            self._battery.recharge()

    def assign_operator(self, operator: Operator):
        self._operator = operator
        print(
            f"Robot {self._robot_id} has been assigned a new operator: {self._operator.name}"
        )

    def perform_task(self):
        print(f"Robot {self._robot_id} is performing a task")
