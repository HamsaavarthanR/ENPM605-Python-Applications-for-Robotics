# MAIN #

# Import modules
import os
import sys

path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(path)

# Import modules
from models.sensor.lidar_interface import LiDAR
from models.sensor.camera_interface import Camera
from models.sensor.sensorsystem_interface import SensorSyestem

from models.robot.planetaryrover_interface import PlanetaryRobot
from models.robot.orbitalrobot_interface import OrbitalRobot
from models.robot.roboticfleet_interface import RoboticsFleet


"""
MAIN function to demostrate different aspects of the program by testing.

"""
def main():
    # SPACE ROBOT 1 #
    # - Create a `PlanetaryRover` robot:
    #   - Name: `MarsExplorer`
    #   - Mobility: `wheels`
    #   - Terrain: `rocky`
    #   - Sensors:
    #     - LiDAR: `"X-500"`, `100.0`, `0.5`
    #     - LiDAR: `"OS1"`, `20.5`, `1.5`
    #     - Camera: `"CamPro"`, `50.0`, `20`

    # Create sensor system for Robot 1
    sensor_sys1 = SensorSyestem(
        [
            LiDAR("X-500", 100.0, 0.5),
            LiDAR("OS1", 20.5, 1.5),
            Camera("CamPro", 50.0, 20),
        ]
    )
    marsexp = PlanetaryRobot("MarsExplorer", sensor_sys1, "wheels", "rocky")

    # SPACE ROBOT 2 #
    # - Create an `OrbitalRobotRover` robot:
    #   - Name: `OrbitFixer`
    #   - Mobility: `thrusters`
    #   - Altitude: `400.5`
    #   - Sensors:
    #     - LiDAR: `"X-500"`, `100.0`, `0.5`
    #     - Camera: `"CamPro"`, `50.0`, `20`
    #     - Camera: `"OrbitCam"`, `20.0`, `20`

    # Create a sensor system for Robot 2
    sensor_sys2 = SensorSyestem(
        [
            LiDAR("X-500", 100.0, 0.5),
            Camera("CamPro", 50.0, 20),
            Camera("OrbitCam", 20.0, 20),
        ]
    )
    orbfxr = OrbitalRobot("OrbitFixer", sensor_sys2, "thruster", 400.5)

    # - Create a robotic fleet
    robor_fleet = RoboticsFleet([])

    # - Add both robots to the fleet
    print("## ADD ROBOTS ##")
    print(robor_fleet.add_robot(marsexp))
    print(robor_fleet.add_robot(orbfxr))
    print("\n")

    # - Report the status of the fleet
    print("## REPORT STATUS ##")
    sts1 = robor_fleet.report_status()
    for sts in sts1.split(","):
        print(sts)

    # - Activate the sensors for all robots in the fleet
    print("## ACTIVATE SENSORS ##")
    for robot in robor_fleet.robot_get():
        if robot != "":
            infos = robot.activate()
            print(f"Activated: {infos}")

    print("\n")

    # - Deploy mission `mapping`, `collection`, `repair`, and `maintenance`
    print("## DEPLY MISSION ##")
    missions1 = ["mapping", "collection", "repair", "maintanence"]
    for mission in missions1:
        robor_fleet.deploy_mission(mission)
        print("\n")

    # - Remove `OrbitalRobotRover` from the fleet
    print("## REMOVE ROBOTS ##")
    print(robor_fleet.remove_robot(orbfxr))
    print("\n")

    # - Deploy mission `maintenance` and `mapping`
    print("## DEPLY MISSION ##")
    missions2 = ["maintanence", "mapping"]
    for mission in missions2:
        robor_fleet.deploy_mission(mission)
        print("\n")

    # - Report the status of the fleet
    sts2 = robor_fleet.report_status()
    print("## REPORT STATUS ##")
    for sts in sts2.split(","):
        print(sts)


if __name__ == "__main__":
    main()
