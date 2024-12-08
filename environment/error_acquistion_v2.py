import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import functions
from solver.geometry import assign_heights, get_grip_position

import numpy as np
import supercube
from pydrake.all import (
    PiecewisePolynomial,
    RollPitchYaw,
    StartMeshcat,
)
from scipy.spatial.transform import Rotation as R

from pydrake.trajectories import PiecewisePolynomial
from manipulation.meshcat_utils import AddMeshcatTriad
from solver.geometry import assign_heights
from solver.rotation import get_rotation
from urf_rotations import *

pregrasp_distance = 0.07
gripper_position_offset = np.array([0.0, 0.114, 0.0])
gripper_rotation_offset = RollPitchYaw(0, np.pi, np.pi).ToRotationMatrix()
cube_center_position = [0.5, 0.5, 0.25]
face_offset_distance = 0.01

def get_cubie_names ():
    cubie_names = []
    for i in [0,1]:
        for j in [0,1]:
            for k in [0,1]:
                cubie_names.append("cubie_" + str(i) + '_' + str(j) + '_' + str(k))
    return cubie_names

def get_angular_differences (plant, initial_poses, final_poses, state, rotation):
    for cubie_name in get_cubie_names():
        idx = plant.GetBodyByName(cubie_name).index()
        initial_cubie_pose = initial_poses[idx]
        final_cubie_pose = final_poses[idx]

        initial_cubie_rotation = initial_cubie_pose.rotation()
        final_cubie_rotation = final_cubie_pose.rotation()

        cubie = ''.join(cubie_name[-5:].split('_'))
        expected_cubie_rotation = get_rotation(state, rotation, cubie, initial_cubie_rotation)

        rpy1 = [final_cubie_rotation.ToRollPitchYaw().roll_angle(), final_cubie_rotation.ToRollPitchYaw().pitch_angle(), final_cubie_rotation.ToRollPitchYaw().yaw_angle()]
        rpy2 = [expected_cubie_rotation. ToRollPitchYaw().roll_angle(), expected_cubie_rotation. ToRollPitchYaw().pitch_angle(), expected_cubie_rotation. ToRollPitchYaw().yaw_angle()]
        rotation1 = R.from_euler('xyz', rpy1)
        rotation2 = R.from_euler('xyz', rpy2)
        R1 = rotation1.as_matrix()
        R2 = rotation2.as_matrix()

        R_rel = R1 @ np.linalg.inv(R2)
        angle = np.arccos((np.trace(R_rel) - 1) / 2)

        print(cubie_name, angle)
        

def main():
    rotation = 'U'
    scenario_file = "models/urf.rotation.scenario.dmd.yaml"

    meshcat = StartMeshcat()

    builder, plant, scene_graph, station = setup_manipulation_station(meshcat, scenario_file=scenario_file)

    context = plant.CreateDefaultContext()
    gripper = plant.GetBodyByName("body")
    initial_pose = plant.EvalBodyPoseInWorld(context, gripper)

    pocket_cube = supercube.PocketCube()
    cubie_heights = assign_heights([0.02, 0.03, 0.02, 0.03, 0.04, 0.04])
    current_state = pocket_cube.get_state()

    trajs = make_gripper_trajectory(initial_pose,
                                    rotation,
                                    current_state,
                                    cubie_heights)
    entry_duration = 2.0
    grip_duration = 1.0
    rotate_duration = 5.0
    exit_duration = 2.0
    durations = [entry_duration, grip_duration, rotate_duration, exit_duration]

    total_duration = sum(durations)
    interval_count = int(total_duration * 2 + 1)

    t_lst = np.linspace(0, total_duration, interval_count)
    pose_lst = []
    for t in t_lst:
        pose = InterpolatePose(t, 
                               rotation, 
                               trajs,
                               current_state, 
                               cubie_heights, 
                               durations)
        AddMeshcatTriad(meshcat, path=str(t), X_PT = pose, opacity=0.02)
        pose_lst.append(pose)

    gripper_t_lst = np.array([0.0, 
                              entry_duration, 
                              entry_duration + grip_duration, 
                              entry_duration + grip_duration + rotate_duration,
                              entry_duration + grip_duration + rotate_duration + exit_duration])
    gripper_knots = np.array([0.08, 
                              0.08, 
                              0.00, 
                              0.00,
                              0.08]).reshape(1, 5)
    g_traj = PiecewisePolynomial.FirstOrderHold(gripper_t_lst, gripper_knots)

    initial_poses_all = plant.get_body_poses_output_port().Eval(context)
    
    q_knots = np.array(create_q_knots(pose_lst, scenario_file))
    q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, q_knots[:, 0:7].T)
    simulator= BuildAndSimulateTrajectory(builder, station, q_traj, g_traj, meshcat, total_duration)
    
    final_poses_all = plant.get_body_poses_output_port().Eval(plant.GetMyContextFromRoot(simulator.get_mutable_context()))

    get_angular_differences(plant, initial_poses_all, final_poses_all, current_state, rotation)

if __name__ == "__main__":
    main()