import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import functions
from solver.geometry import assign_heights, get_grip_position

import numpy as np
import json
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
from environment.urf_rotations import *
from error_analysis import *
from cube_length_cubie_ratio_setup import cube_lengths, cubie_ratios

pregrasp_distance = 0.07
gripper_position_offset = np.array([0.0, 0.114, 0.0])
gripper_rotation_offset = RollPitchYaw(0, np.pi, np.pi).ToRotationMatrix()
cube_center_position = [0.5, 0.5, 0.25]
face_offset_distance = 0.01

def main():
    for i, cube_length in enumerate(cube_lengths):
        for j, cubie_ratio in enumerate(cubie_ratios):
            for rotation in ['U', 'F', 'R', 'U\'', 'F\'', 'R\'']:
                if not(cube_length == 0.045 and cubie_ratio == 0.005 and rotation == 'U'):
                    continue
                # try:
                scenario_file = "models/cube_length_cubie_ratio/l_" +str(i) + "_r_" + str(j) + "scenario.dmd.yaml"

                meshcat = StartMeshcat()
                # meshcat = None

                builder, plant, scene_graph, station = setup_manipulation_station(scenario_file, meshcat)

                context = plant.CreateDefaultContext()
                gripper = plant.GetBodyByName("body")
                initial_pose = plant.EvalBodyPoseInWorld(context, gripper)

                pocket_cube = supercube.PocketCube()
                # cubie_heights = assign_heights([0.02, 0.03, 0.02, 0.03, 0.04, 0.04])
                cubie_heights = assign_heights([cube_length - cubie_ratio,
                                                cube_length,
                                                cube_length - cubie_ratio,
                                                cube_length, 
                                                cube_length + cubie_ratio,
                                                cube_length + cubie_ratio])
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
                    if meshcat != None:
                        AddMeshcatTriad(meshcat, path=str(t), X_PT = pose, opacity=0.02)
                    print(t)
                    print(pose.translation())
                    print('\n')
                    pose_lst.append(pose)

                gripper_t_lst = np.array([0.0, 
                                        entry_duration, 
                                        entry_duration + grip_duration, 
                                        entry_duration + grip_duration + rotate_duration,
                                        entry_duration + grip_duration + rotate_duration + exit_duration])
                gripper_knots = np.array([0.10, 
                                        0.10, 
                                        0.00, 
                                        0.00,
                                        0.10]).reshape(1, 5)
                g_traj = PiecewisePolynomial.FirstOrderHold(gripper_t_lst, gripper_knots)

                initial_poses_all = plant.get_body_poses_output_port().Eval(context)
                
                q_knots = np.array(create_q_knots(pose_lst, scenario_file))
                q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, q_knots[:, 0:7].T)
                simulator= BuildAndSimulateTrajectory(builder, station, q_traj, g_traj, meshcat, total_duration)
                
                final_poses_all = plant.get_body_poses_output_port().Eval(plant.GetMyContextFromRoot(simulator.get_mutable_context()))

                result = get_angular_differences(plant, initial_poses_all, final_poses_all, current_state, rotation)
                filename = 'experiment/cube_length_cubie_ratio_results/' + 'l_' + str(i) + '_r_' + str(j) + '_' + rotation + '.json'
                with open(filename, "w") as file:
                    json.dump(result, file)
                # except:
                #     print(cube_length, cubie_ratio, rotation)
                #     print("failed!")


if __name__ == "__main__":
    main()