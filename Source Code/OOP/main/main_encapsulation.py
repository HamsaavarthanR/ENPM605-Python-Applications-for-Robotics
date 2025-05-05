import sys
import os.path

path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(path)

from warehouse.robot_encapsulation import Robot

if __name__ == "__main__":
    titan = Robot(robot_id="TN-8800", model="Titan", battery_charge_level=20.0)
    print(titan)
    print("=" * 20)
    nomad = Robot(robot_id="NMD-427", model="Nomad", battery_charge_level=50.0)
    print(nomad)
