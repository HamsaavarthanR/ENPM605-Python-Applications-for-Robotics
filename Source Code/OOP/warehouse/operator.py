
class Operator:

    def __init__(self, name: str) -> None:
        if not self._validate_name(name):
            raise ValueError("Invalid name. Name must be a non-empty string.")
        self._name = name
        
    @property
    def name(self):
        """
        Name of the instructor
        """
        return self._name
    
    
    @staticmethod
    def _validate_name(name):
        """Validates that the name is a non-empty string."""
        return isinstance(name, str) and bool(name.strip())
    
    def give_instruction(self, robot, instruction, time_required=1):
        """Sends an instruction to a robot if the operator is assigned to it."""
        if robot.operator != self:
            print(f"Error: {self.name} is not assigned to operate {robot.model} Robot {robot.robot_id}.")
            return
        
        print(f"Operator {self.name} instructs {robot.model} Robot {robot.robot_id}: {instruction}")
        robot.move(instruction, time_required)  # Automatically moves the robot
        