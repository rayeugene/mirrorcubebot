import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from pydrake.all import (
    RollPitchYaw,
)
from scipy.spatial.transform import Rotation as R
from solver.rotation import get_rotation
from environment.urf_rotations import *

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
    cubie_angular_errors = {}
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

        cubie_angular_errors[cubie_name] = angle
    return cubie_angular_errors