import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

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
    TrajectorySource
)
from pydrake.multibody import inverse_kinematics
from pydrake.trajectories import PiecewisePolynomial

from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.scenarios import AddMultibodyTriad
from manipulation.station import LoadScenario, MakeHardwareStation, MakeMultibodyPlant

from solver.geometry import assign_heights, get_grip_position

from experiment.error_analysis import *

pregrasp_distance = 0.07
gripper_position_offset = np.array([0.0, 0.114, 0.0])
gripper_rotation_offset = RollPitchYaw(0, np.pi, np.pi).ToRotationMatrix()
cube_center_position = [0.5, 0.5, 0.25]
face_offset_distance = 0.01

def compute_handle_pose(
    t,
    cube_center_position, 
    current_state, 
    cubie_heights, 
    rotation,
    grasping_vertical_offset = 0.01
):
    """
    Compute the position and rotation of a handle based on cube parameters, rotation, and time.

    Parameters:
        t (float): 
            Interpolation parameter between 0 and 1. Determines the progress of the rotation.
        cube_center_position (list or numpy array): 
            The 3D coordinates of the cube's center position.
        current_state (object): 
            Current state of the cube, providing contextual data for the handle's position.
        cubie_heights (list or numpy array): 
            Heights of individual cubies, used for determining grip positioning.
        rotation (str): 
            The rotation direction (e.g., 'U', 'U\'', 'F', 'F\'', 'R', 'R\'').

    Returns:
        tuple: 
            - numpy array: Handle position in world coordinates (p_Whandle).
            - float: Rotation angle (theta) after interpolation.
    """
    rotation_face = rotation[0]

    match(rotation_face):
        case 'U' : 
            face_offset  = np.array([0.0, 0.0, face_offset_distance])
            angle_start = np.pi / 2
        case 'F' : 
            face_offset  = np.array([-face_offset_distance, 0.0, 0.0])
            angle_start = np.pi / 2
        case 'R' : 
            face_offset  = np.array([0.0, -face_offset_distance, 0.0])
            angle_start = np.pi

    angle_end = (angle_start + np.pi/2 if rotation in ['U\'', 'F\'', 'R'] 
                 else angle_start - np.pi/2)
    
    if rotation == 'R':
        angle_end -= (np.pi / 2) / 64
    elif rotation == 'R\'':
        angle_end += (np.pi / 2) / 64

    vertical_offset, horizontal_offset = get_grip_position(current_state, cubie_heights, rotation, vertical_offset = grasping_vertical_offset)
    distance_from_axis = np.sqrt(vertical_offset ** 2 + horizontal_offset ** 2)

    adjusted_t = min(max(t,0), 1)

    match(rotation_face):
        case 'U' : initial_angle = np.arctan2(-vertical_offset, horizontal_offset)
        case 'F' : initial_angle = np.arctan2(vertical_offset, horizontal_offset)
        case 'R' : initial_angle = np.arctan2(horizontal_offset, vertical_offset)

    match(rotation):
        case 'U' | 'F\'' | 'R' : angle = initial_angle - adjusted_t * np.pi/2
        case 'U\'' | 'F' | 'R\'' : angle = initial_angle + adjusted_t * np.pi/2

    match (rotation_face):
        case 'U' : face_center_position = np.array([distance_from_axis * np.cos(angle), distance_from_axis * np.sin(angle), 0])
        case 'F' : face_center_position = np.array([0, distance_from_axis * np.cos(angle), distance_from_axis * np.sin(angle)])
        case 'R' : face_center_position = np.array([distance_from_axis * np.cos(angle), 0, distance_from_axis * np.sin(angle)])
    
    p_Whandle = np.add(np.add(cube_center_position, face_offset), face_center_position)
    theta = angle_start + (angle_end - angle_start) * adjusted_t

    if t < 0 or t > 1:
        pregrasp_offset = np.array([pregrasp_distance * np.sign(x) for x in face_offset])
        p_Whandle += pregrasp_offset

    return p_Whandle, theta

def make_gripper_trajectory(initial_pose,
                            rotation,
                            current_state,
                            cubie_heights,
                            grasping_vertical_offset = 0.01):
    """
    Generates trajectories for a robotic gripper's entry and exit maneuvers.

    Parameters:
        initial_pose (RigidTransform): The starting pose of the gripper.
        rotation (RotationMatrix): The rotation to apply during the trajectory.
        current_state (dict): Current state of the robot, potentially used to determine pose adjustments.
        cubie_heights (list or array): Heights of cuboids or objects in the workspace for trajectory calculations.

    Returns:
        tuple: A tuple containing four trajectory objects:
            - entry_orientation_traj: Orientation trajectory for the entry phase.
            - entry_position_traj: Position trajectory for the entry phase.
            - exit_orientation_traj: Orientation trajectory for the exit phase.
            - exit_position_traj: Position trajectory for the exit phase.
    """
    pregrasp_pose = InterpolatePoseRotate(-1.0, rotation, current_state, cubie_heights, grasping_vertical_offset)
    initial_grasp_pose = InterpolatePoseRotate(0.0, rotation, current_state, cubie_heights, grasping_vertical_offset)
    final_grasp_pose = InterpolatePoseRotate(1.0, rotation, current_state, cubie_heights, grasping_vertical_offset)
    postgrasp_pose = InterpolatePoseRotate(2.0, rotation, current_state, cubie_heights, grasping_vertical_offset)
    ready_pose = ReadyPose()

    # Entry orientation trajectory
    entry_orientation_traj = PiecewiseQuaternionSlerp()
    entry_orientation_traj.Append(0.0, initial_pose.rotation())
    entry_orientation_traj.Append(0.8, pregrasp_pose.rotation())
    entry_orientation_traj.Append(1.0, initial_grasp_pose.rotation())

    # Entry position trajectory
    entry_position_traj = PiecewisePolynomial.FirstOrderHold(
        [0.0, 0.8, 1.0],
        np.vstack([
            initial_pose.translation(),
            pregrasp_pose.translation(),
            initial_grasp_pose.translation()
        ]).T,
    )

    # Exit orientation trajectory
    exit_orientation_traj = PiecewiseQuaternionSlerp()
    exit_orientation_traj.Append(0.0, final_grasp_pose.rotation())
    exit_orientation_traj.Append(0.5, final_grasp_pose.rotation())
    exit_orientation_traj.Append(1.0, ready_pose.rotation())

    # Exit position trajectory
    exit_position_traj = PiecewisePolynomial.FirstOrderHold(
        [0.0, 0.1, 0.5, 1.0],
        np.vstack([
            final_grasp_pose.translation(),
            final_grasp_pose.translation(),
            postgrasp_pose.translation(),
            ready_pose.translation()
        ]).T,
    )

    return entry_orientation_traj, entry_position_traj, exit_orientation_traj, exit_position_traj

def InterpolatePoseRotate(
        t: float, 
        rotation: str, 
        current_state, 
        cubie_heights,
        grasping_vertical_offset = 0.01) -> RigidTransform:
    """
    Interpolates the pose for opening doors based on the rotation type and time.

    Parameters:
        t (float): Interpolation parameter (0 to 1). Negative values indicate pre-positioning.
        rotation (str): Rotation type ('U', 'U\'', 'F', 'F\'', 'R', 'R\'').

    Returns:
        RigidTransform: The interpolated rigid transform.
    """
    p_Whandle, theta = compute_handle_pose(t,
                                           cube_center_position, 
                                           current_state, 
                                           cubie_heights, 
                                           rotation,
                                           grasping_vertical_offset)

    # Determine roll-pitch-yaw order based on rotation
    rotation_face = rotation[0]
    if rotation_face == 'U':
        R_Whandle = RollPitchYaw(np.pi / 2, 0, theta).ToRotationMatrix()
    elif rotation_face == 'F':
        R_Whandle = RollPitchYaw(0, theta, np.pi / 2).ToRotationMatrix()
    elif rotation_face == 'R':
        R_Whandle = RollPitchYaw(np.pi, theta, 0).ToRotationMatrix()
    else:
        raise ValueError(f"Invalid rotation type: {rotation}")

    X_Whandle = RigidTransform(R_Whandle, p_Whandle)

    # Add a gripper offset
    p_handleG = gripper_position_offset
    R_handleG = gripper_rotation_offset
    X_handleG = RigidTransform(R_handleG, p_handleG)

    return X_Whandle.multiply(X_handleG)

def InterpolatePoseEntry(t, entry_traj_rotation, entry_traj_translation):
    return RigidTransform(
        RotationMatrix(entry_traj_rotation.orientation(t)),
        entry_traj_translation.value(t),
    )

def InterpolatePoseExit(t, exit_traj_rotation, exit_traj_translation):
    return RigidTransform(
        RotationMatrix(exit_traj_rotation.orientation(t)),
        exit_traj_translation.value(t),
    )

def ReadyPose():
    R_Whandle = RollPitchYaw(np.pi / 2, 0, np.pi/2).ToRotationMatrix()
    p_Whandle = [0.35, 0.35, 0.35]

    X_Whandle = RigidTransform(R_Whandle, p_Whandle)

    # Add a gripper offset
    p_handleG = gripper_position_offset
    R_handleG = gripper_rotation_offset
    X_handleG = RigidTransform(R_handleG, p_handleG)

    return X_Whandle.multiply(X_handleG)

def InterpolatePose(t, 
                    rotation, 
                    trajs,
                    current_state, 
                    cubie_heights,
                    durations,
                    grasping_vertical_offset
                    ):
    entry_duration, grip_duration, rotate_duration, exit_duration = durations
    entry_traj_rotation, entry_traj_translation, exit_traj_rotation, exit_traj_translation = trajs
    if t < entry_duration:
        return InterpolatePoseEntry(t / entry_duration if entry_duration != 0 else 0.0, 
                                    entry_traj_rotation, 
                                    entry_traj_translation)
    elif t < entry_duration + grip_duration:
        return InterpolatePoseEntry(1.0, 
                                    entry_traj_rotation, 
                                    entry_traj_translation)
    elif t < entry_duration + grip_duration + rotate_duration:
        return InterpolatePoseRotate((t - (entry_duration + grip_duration)) / rotate_duration, 
                                     rotation, 
                                     current_state, 
                                     cubie_heights,
                                     grasping_vertical_offset)
    elif t < entry_duration + grip_duration + rotate_duration + exit_duration:
        return InterpolatePoseExit((t - (entry_duration + grip_duration + rotate_duration)) / exit_duration,
                                   exit_traj_rotation,
                                   exit_traj_translation)
    else: 
        return InterpolatePoseExit(1.0,
                                   exit_traj_rotation,
                                   exit_traj_translation)


def CreateIiwaControllerPlant(scenario_file):
    """creates plant that includes only the robot and gripper, used for controllers."""
    scenario = LoadScenario(filename=scenario_file)
    plant_robot = MakeMultibodyPlant(
        scenario=scenario, model_instance_names=["iiwa", "wsg"]
    )
    link_frame_indices = []
    for i in range(8):
        link_frame_indices.append(
            plant_robot.GetFrameByName("iiwa_link_" + str(i)).index()
        )

    return plant_robot, link_frame_indices

def setup_manipulation_station(scenario_file, meshcat = None):
    builder = DiagramBuilder()
    scenario = LoadScenario(filename=scenario_file)
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat, package_xmls=[os.getcwd() + "/package.xml"]))
    plant = station.GetSubsystemByName("plant")
    scene_graph = station.GetSubsystemByName("scene_graph")
    AddMultibodyTriad(plant.GetFrameByName("body"), scene_graph)

    return builder, plant, scene_graph, station

def BuildAndSimulateTrajectory(builder, station, q_traj, g_traj, meshcat = None, duration=0.01):
    """Simulate trajectory for manipulation station.
    @param q_traj: Trajectory class used to initialize TrajectorySource for joints.
    @param g_traj: Trajectory class used to initialize TrajectorySource for gripper.
    """
    q_traj_system = builder.AddSystem(TrajectorySource(q_traj))
    g_traj_system = builder.AddSystem(TrajectorySource(g_traj))

    builder.Connect(
        q_traj_system.get_output_port(), station.GetInputPort("iiwa.position")
    )
    builder.Connect(
        g_traj_system.get_output_port(), station.GetInputPort("wsg.position")
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    if meshcat != None:
        meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(duration)
    if meshcat != None:
        meshcat.PublishRecording()

    return simulator

def create_q_knots(pose_lst, scenario_file):
    """Convert end-effector pose list to joint position list using series of
    InverseKinematics problems. Note that q is 9-dimensional because the last 2 dimensions
    contain gripper joints, but these should not matter to the constraints.
    @param: pose_lst (python list): post_lst[i] contains keyframe X_WG at index i.
    @return: q_knots (python_list): q_knots[i] contains IK solution that will give f(q_knots[i]) \approx pose_lst[i].
    """
    q_knots = []
    plant, _ = CreateIiwaControllerPlant(scenario_file)
    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName("body")
    q_nominal = np.array(
        [0.0, 0.6, 0.0, -1.75, 0.0, 1.0, 0.0, 0.0, 0.0]
    )  # nominal joint angles for joint-centering.

    def AddOrientationConstraint(ik, R_WG, bounds):
        """Add orientation constraint to the ik problem. Implements an inequality
        constraint where the axis-angle difference between f_R(q) and R_WG must be
        within bounds. Can be translated to:
        ik.prog().AddBoundingBoxConstraint(angle_diff(f_R(q), R_WG), -bounds, bounds)
        """
        ik.AddOrientationConstraint(
            frameAbar=world_frame,
            R_AbarA=R_WG,
            frameBbar=gripper_frame,
            R_BbarB=RotationMatrix(),
            theta_bound=bounds,
        )

    def AddPositionConstraint(ik, p_WG_lower, p_WG_upper):
        """Add position constraint to the ik problem. Implements an inequality
        constraint where f_p(q) must lie between p_WG_lower and p_WG_upper. Can be
        translated to
        ik.prog().AddBoundingBoxConstraint(f_p(q), p_WG_lower, p_WG_upper)
        """
        ik.AddPositionConstraint(
            frameA=world_frame,
            frameB=gripper_frame,
            p_BQ=np.zeros(3),
            p_AQ_lower=p_WG_lower,
            p_AQ_upper=p_WG_upper,
        )

    for i in range(len(pose_lst)):
        ik = inverse_kinematics.InverseKinematics(plant)
        q_variables = ik.q()
        prog = ik.prog()

        # Target position and orientation from pose X_WG
        X_WG = pose_lst[i]
        p_WG = X_WG.translation()
        R_WG = X_WG.rotation()

        # Position constraint on x and z
        positional_tolerance = 0.0001
        p_WG_lower = np.array([p_WG[0]-positional_tolerance, p_WG[1]-positional_tolerance, p_WG[2]-positional_tolerance])
        p_WG_upper = np.array([p_WG[0]+positional_tolerance, p_WG[1]+positional_tolerance, p_WG[2]+positional_tolerance])
        AddPositionConstraint(ik, p_WG_lower, p_WG_upper)

        # Orientation constraint to ensure pitch alignment towards y-axis
        theta_bound = np.pi / 1800
        AddOrientationConstraint(ik, R_WG, bounds=theta_bound)

        # Add joint-centering cost to maintain natural joint positions
        prog.AddQuadraticErrorCost(np.eye(len(q_variables)), q_nominal, q_variables)

        # Set initial guess
        if i == 0:
            prog.SetInitialGuess(q_variables, q_nominal)
        else:
            prog.SetInitialGuess(q_variables, q_knots[-1])  

        result = Solve(prog)
        #assert result.is_success()
        q_knots.append(result.GetSolution(q_variables))

    return q_knots

def print_pose(pose):
    translation = pose.translation()
    print(f"x: {translation[0]:.3f}, y: {translation[1]:.3f}, z: {translation[2]:.3f}")
    rotation = pose.rotation().ToRollPitchYaw()
    print(f"Roll: {rotation.roll_angle() / np.pi * 180:.1f}, Pitch: {rotation.pitch_angle() / np.pi * 180:.1f}, Yaw: {rotation.yaw_angle() / np.pi * 180:.1f}")

def get_cubie_names ():
    cubie_names = []
    for i in [0,1]:
        for j in [0,1]:
            for k in [0,1]:
                cubie_names.append("cubie_" + str(i) + '_' + str(j) + '_' + str(k))
    return cubie_names

def main():
    rotation = 'U'
    scenario_file = "models/urf.rotation.scenario.dmd.yaml"

    meshcat = StartMeshcat()
    #meshcat = None

    builder, plant, scene_graph, station = setup_manipulation_station(scenario_file, meshcat)

    context = plant.CreateDefaultContext()
    gripper = plant.GetBodyByName("body")
    initial_pose = plant.EvalBodyPoseInWorld(context, gripper)

    pocket_cube = supercube.PocketCube()
    cubie_heights = assign_heights([0.02, 0.03, 0.02, 0.03, 0.04, 0.04])
    current_state = pocket_cube.get_state()

    trajs = make_gripper_trajectory(initial_pose,
                                    rotation,
                                    current_state,
                                    cubie_heights,
                                    grasping_vertical_offset = 0.01)
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
                               durations,
                               grasping_vertical_offset = 0.01)
        if meshcat != None:
            AddMeshcatTriad(meshcat, path=str(t), X_PT = pose, opacity=0.02)
        pose_lst.append(pose)
        # print(t)
        # print(pose.translation())
        # print('\n')

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

    initial_X_WB_all = plant.get_body_poses_output_port().Eval(context)
    
    q_knots = np.array(create_q_knots(pose_lst, scenario_file))
    q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, q_knots[:, 0:7].T)
    simulator= BuildAndSimulateTrajectory(builder, station, q_traj, g_traj, meshcat, total_duration)
    
    final_X_WB_all = plant.get_body_poses_output_port().Eval(plant.GetMyContextFromRoot(simulator.get_mutable_context()))

    result = get_angular_differences(plant, initial_X_WB_all, final_X_WB_all, current_state, rotation)

    print(result)

    # for cubie_name in get_cubie_names():
    #     idx = plant.GetBodyByName(cubie_name).index()
    #     initial_pose = initial_X_WB_all[idx]
    #     final_pose = final_X_WB_all[idx]
    #     print(cubie_name)
    #     print('Before')
    #     print_pose(initial_pose)
    #     print('After')
    #     print_pose(final_pose)
    #     print('\n')

if __name__ == "__main__":
    main()