import argparse
from utils import SimpleRobotSimulation
from onshape_to_robot.simulation import Simulation  # type: ignore
import kinematics
import pybullet as p  # type: ignore

# === Argument parser pour choisir le mode ===
parser = argparse.ArgumentParser()
parser.add_argument("--mode", "-m", type=str, default="marche",
                    help="Modes disponibles : marche, tourne_droite, tourne_gauche, danse")
args = parser.parse_args()

# === Initialisation de la simulation ===
robotPath = "phantomx_description/urdf/phantomx.urdf"
sim = Simulation(robotPath, gui=True, panels=True, useUrdfInertia=False)
robot = SimpleRobotSimulation(sim)
robot.init()

# === Boucle principale ===
while True:
    robot.tick_read_and_write()

    mode = args.mode

    if mode == "marche":
        extra_angle = kinematics.get_extra_angle()
        if extra_angle is not None:
            for l in [1, 3, 5]:
                thetas = kinematics.triangle(0, -0.05, 0.03, 0.08, sim.t, leg_id=l)
                for m in range(3):
                    robot.legs[l][m].goal_position = thetas[m]
            for l in [2, 4, 6]:
                thetas = kinematics.triangle(0, -0.05, 0.03, 0.08, sim.t + 1, leg_id=l)
                for m in range(3):
                    robot.legs[l][m].goal_position = thetas[m]

    elif mode == "tourne_droite":
        for l in range(1, 7):
            phase_offset = 1 if l % 2 == 0 else 0
            thetas = kinematics.triangle2(0.2, -0.05, 0.03, 0.08, sim.t + phase_offset)
            for m in range(3):
                robot.legs[l][m].goal_position = thetas[m]

    elif mode == "tourne_gauche":
        for l in range(1, 7):
            phase_offset = 1 if l % 2 == 0 else 0
            thetas = kinematics.triangle(0.2, -0.05, 0.03, 0.08, sim.t + phase_offset)
            for m in range(3):
                robot.legs[l][m].goal_position = thetas[m]

    elif mode == "danse":

        index_patte1 = [1,3,5]
        index_patte2 = [2,4,6]

        for l in index_patte1:
            thetas = kinematics.triangle2(0, -0.05, 0, 0.08, sim.t*3, leg_id=l)
            for m in range(0,3):
                robot.legs[l][m].goal_position = thetas[m]

        for l in index_patte2:
            thetas = kinematics.triangle(0, -0.05, 0, 0.08, sim.t*3+1, leg_id= l)
            for m in range(0,3):
                robot.legs[l][m].goal_position = thetas[m]

    else:
        print(f"Mode inconnu : {mode}")

    sim.tick()
