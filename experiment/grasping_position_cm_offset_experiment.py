import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import functions
from solver.geometry import assign_heights, get_grip_position, get_center_of_mass, get_length

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
from grasping_position_cm_offset_setup import cubie_ratios

pregrasp_distance = 0.07
gripper_position_offset = np.array([0.0, 0.114, 0.0])
gripper_rotation_offset = RollPitchYaw(0, np.pi, np.pi).ToRotationMatrix()
cube_center_position = [0.5, 0.5, 0.25]
face_offset_distance = 0.01

fps = 3
rotate_duration = 5.0

grasping_offsets = [0.010, 0.011, 0.012, 0.013, 0.014, 0.015]

def main():
    result = []
    for i, grasping_offset in enumerate(grasping_offsets):
        for j, cubie_ratio in enumerate(cubie_ratios):
            for rotation in ['U', 'F', 'R', 'U\'', 'F\'', 'R\'']:
                try:
                    scenario_file = "models/grasping_position_cm_offset/mirror_cube_r_" + str(j) + ".scenario.dmd.yaml"



                    # meshcat = StartMeshcat()
                    meshcat = None

                    builder, plant, scene_graph, station = setup_manipulation_station(scenario_file, meshcat)

                    context = plant.CreateDefaultContext()
                    gripper = plant.GetBodyByName("body")
                    initial_pose = plant.EvalBodyPoseInWorld(context, gripper)

                    pocket_cube = supercube.PocketCube()
                    cubie_heights = assign_heights([0.02, 0.03, 0.02, 0.03, 0.04, 0.04])
                    current_state = pocket_cube.get_state()

                    def get_grip_position(state, heights, rotation, vertical_offset = 0.01)

                    grip_coordinates = get_grip_position(current_state, cubie_heights, rotation, grasping_offset) 
                    cm_coordinates = get_center_of_mass(current_state, cubie_heights, rotation)
                    relative_coordinates = [x-y for x, y in zip(grip_coordinates, cm_coordinates)]

                    pg_distance = get_length(grip_coordinates)# pivot to grapsing center
                    pm_distance = get_length(cm_coordinates)# pivot to center of mass 
                    gm_distance = get_length(relative_coordinates)# grapsing center to center of mass

                    trajs = make_gripper_trajectory(initial_pose,
                                                    rotation,
                                                    current_state,
                                                    cubie_heights,
                                                    grasping_offset)
                    entry_duration = 2.0
                    grip_duration = 1.0
                    rotate_duration = rotate_duration
                    exit_duration = 2.0
                    durations = [entry_duration, grip_duration, rotate_duration, exit_duration]

                    total_duration = sum(durations)
                    interval_count = int(total_duration * fps + 1)

                    t_lst = np.linspace(0, total_duration, interval_count)
                    pose_lst = []
                    for t in t_lst:
                        pose = InterpolatePose(t, 
                                            rotation, 
                                            trajs,
                                            current_state, 
                                            cubie_heights, 
                                            durations,
                                            grasping_offset)
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

                    angular_differences = list(get_angular_differences(plant, initial_poses_all, final_poses_all, current_state, rotation).values())
                    angular_difference = sum(angular_differences)/len(angular_differences)
                    result.append([pg_distance, pm_distance, gm_distance, rotation, angular_difference])

                    print(grasping_offset, cubie_ratio, rotation, angular_difference)            
                except:
                    print(grasping_offset, cubie_ratio, rotation)
                    print("failed!")

    filename = 'experiment/grasping_position_cm_offset_results/result.json'
    with open(filename, "w") as file:
        json.dump(result, file)


if __name__ == "__main__":
    main()