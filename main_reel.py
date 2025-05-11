import pypot.dynamixel
import time
import math
from utils import *
import time
from kinematics import *
import math
import sys
from signal import signal, SIGINT
import traceback
import os


def main():
    # angles = computeIK(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), 51, 64, 93)
    # print("Angles : " + str(angles))

    ports = pypot.dynamixel.get_available_ports()
    dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)

    robot = SimpleRobot(dxl_io)
    robot.init()
    time.sleep(0.1)
    robot.enable_torque()

    # Defining the shutdown function here so it has visibility over the robot variable
    def shutdown(signal_received, frame):
        # Handle any cleanup here
        print("SIGINT or CTRL-C detected. Setting motors to compliant and exiting")
        robot.disable_torque()
        print("Done ticking. Exiting.")
        sys.exit()
        # Brutal exit
        # os._exit(1)

    # Tell Python to run the shutdown() function when SIGINT is recieved
    signal(SIGINT, shutdown)
    try:
        params = Parameters(
            freq=50,
            speed=1,
            z=80,
            travelDistancePerStep=80,
            lateralDistance=110,
            frontDistance=87,
            frontStart=32,
            method="minJerk",
        )

        print("Setting initial position")
        # initPositionForWalk(robot, params)
        setPositionToRobot(0, 0, 0, robot, params)
        robot.smooth_tick_read_and_write(3, verbose=True)
        print("Init position reached")
        keep_going = True
        while keep_going:
            print("A vous de bosser")
            time.sleep(2)
        return
    except Exception as e:
        track = traceback.format_exc()
        print(track)
    finally:
        shutdown(None, None)


print("A new day dawns")
main()
print("Done !")
