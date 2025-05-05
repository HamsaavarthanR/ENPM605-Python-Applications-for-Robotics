import sys
import os.path

path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(path)

# from warehouse.robot_practice import Robot
from warehouse.robot import Robot
from warehouse.operator import Operator
from warehouse.battery import Battery
from warehouse.scanner_robot import ScannerRobot
from warehouse.sorter_robot import SorterRobot
from warehouse.carrier_robot import CarrierRobot


if __name__ == "__main__":
    # -----------------
    # Class method: Access class attributes
    # -----------------
    # titan = Robot(robot_id="TN-8800", model="Titan", battery_capacity=20.0)
    # print(Robot.get_total_robots()) # 1
    # nomad = Robot(robot_id="NMD-427", model="Nomad", battery_capacity=50.0)
    # print(Robot.get_total_robots()) # 2

    # -----------------
    # Class method: Object factory
    # -----------------
    # try:
    #     robot1 = Robot.build_from_id("TN-8800", 75)
    #     print(robot1.robot_id, "->", robot1.model)  # TN-8800 -> Titan

    #     robot2 = Robot.build_from_id("NMD-500", 60)
    #     print(robot2.robot_id, "->", robot2.model)  # NMD-500 -> Nomad

    #     robot3 = Robot.build_from_id("UNKNOWN-9999", 50)  # Should raise an error
    # except ValueError as e:
    #     print(f"Error: {e}")
    
    # -----------------
    # Static method
    # -----------------
    # titan = Robot(robot_id="T4N-8800", model="Titan", battery_capacity=20.0)

    
    # -----------------
    # Aggregation
    # -----------------
    # titan = Robot(robot_id="TN-8800", model="Titan", battery_capacity=100)
    # john = Operator("John Doe")
    # titan.assign_operator(john)
    # john.give_instruction(titan, "go to workcell")

    # nomad = Robot(robot_id="NMD-427", model="Nomad", battery_capacity=50)
    # john.give_instruction(nomad, "go to workcell")

    # -----------------
    # testing inheritance
    # -----------------
    # scanner_robot = ScannerRobot("SCN-3323", "Scanner", 100, 5.0)
    # sorter_robot = SorterRobot("SOR-34423", "Sorter", 80, 15)
    # carrier_robot = CarrierRobot("CAR-101", "Carrier", 70, 50.7)
    
    # -----------------
    # method overriding
    # -----------------
    # scanner_robot = ScannerRobot("SCN-3323", "Scanner", 100, 5.0)
    # sorter_robot = SorterRobot("SOR-34423", "Sorter", 80, 15)
    # carrier_robot = CarrierRobot("CAR-101", "Carrier", 70, 50.7)
    
    # robots = [scanner_robot, sorter_robot, carrier_robot]
    
    # for robot in robots:
    #     robot.perform_task()
    
    
