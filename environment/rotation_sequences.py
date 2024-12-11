import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import functions
from solver.geometry import assign_heights, get_grip_position

import numpy as np
import supercube
from pydrake.all import (
    DiagramBuilder,
    PiecewisePolynomial,
    PiecewiseQuaternionSlerp,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
    Solve,
    StartMeshcat,
    TrajectorySource,
)
from pydrake.multibody import inverse_kinematics
from pydrake.trajectories import PiecewisePolynomial

from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.scenarios import AddMultibodyTriad
from manipulation.station import LoadScenario, MakeHardwareStation, MakeMultibodyPlant

from solver.geometry import *
from urf_rotations import *

pregrasp_distance = 0.07
gripper_position_offset = np.array([0.0, 0.114, 0.0])
gripper_rotation_offset = RollPitchYaw(0, np.pi, np.pi).ToRotationMatrix()
cube_center_position = [0.5, 0.5, 0.25]
face_offset_distance = 0.01

def main():
    rotation_sequence = ['U', 'F', 'R']
    scenario_file = "models/urf.rotation.scenario.dmd.yaml"

    meshcat = StartMeshcat()
    #meshcat = None

    builder, plant, scene_graph, station = setup_manipulation_station(scenario_file, meshcat)
    context = plant.CreateDefaultContext()
    gripper = plant.GetBodyByName("body")
    initial_pose = plant.EvalBodyPoseInWorld(context, gripper)
    
    pocket_cube = supercube.PocketCube()
    cubie_heights = assign_heights([0.02, 0.03, 0.02, 0.03, 0.04, 0.04])

    entry_duration = 2.0
    grip_duration = 1.0
    rotate_duration = 5.0
    exit_duration = 2.0
    durations = [entry_duration, grip_duration, rotate_duration, exit_duration]

    total_duration = sum(durations)
    interval_count = int(total_duration * 2 + 1)

    pose_lst = []
    for rotation in rotation_sequence:
        if len(pose_lst) != 0 : initial_pose = pose_lst[-1]
        current_state = pocket_cube.get_state()
        #print(current_state)

        trajs = make_gripper_trajectory(initial_pose,
                                        rotation,
                                        current_state,
                                        cubie_heights)

        t_lst = np.linspace(0, total_duration, interval_count)
        for t in t_lst:
            pose = InterpolatePose(t, 
                                rotation, 
                                trajs,
                                current_state, 
                                cubie_heights, 
                                durations)
            if meshcat != None:
                AddMeshcatTriad(meshcat, path=str(t), X_PT = pose, opacity=0.02)
            pose_lst.append(pose)
            # print(t)
            # print(pose.translation())
            # print('\n')
        pocket_cube.move(rotation)

        print(get_center_of_mass(pocket_cube.get_state(), cubie_heights, 'U'))


    t_lst = np.linspace(0, total_duration * len(rotation_sequence), interval_count* len(rotation_sequence))

    gripper_t_lst = [0.0]
    for i in range(len(rotation_sequence)):
        gripper_t_lst.append(total_duration * i + entry_duration)
        gripper_t_lst.append(total_duration * i + entry_duration + grip_duration)
        gripper_t_lst.append(total_duration * i + entry_duration + grip_duration + rotate_duration)
        gripper_t_lst.append(total_duration * i + entry_duration + grip_duration + rotate_duration + exit_duration)
    gripper_t_lst = np.array(gripper_t_lst)

    gripper_knots = [0.08]
    for i in range(len(rotation_sequence)):
        gripper_knots.extend([0.08, 0.00, 0.00, 0.08])

    gripper_knots = np.array(gripper_knots).reshape(1, len(gripper_knots))
    
    g_traj = PiecewisePolynomial.FirstOrderHold(gripper_t_lst, gripper_knots)

    q_knots = np.array(create_q_knots(pose_lst, scenario_file))
    q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst , q_knots[:, 0:7].T)
    simulator= BuildAndSimulateTrajectory(builder, station, q_traj, g_traj, meshcat, total_duration * len(rotation_sequence))

if __name__ == "__main__":
    main()