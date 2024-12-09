import numpy as np
import os
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
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.trajectories import PiecewisePolynomial

from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.scenarios import AddMultibodyTriad
from manipulation.station import LoadScenario, MakeHardwareStation, MakeMultibodyPlant

import iiwa_controller
from iiwa_controller import TorqueController

def Setup(parser):
    parser.plant().set_discrete_contact_approximation(
        DiscreteContactApproximation.kLagged
    )


def BuildAndSimulate(ctrl_fun, velocity, duration):
    builder = DiagramBuilder()

    scenario = LoadScenario(
        filename=FindResource("models/planar_manipulation_station.scenario.yaml")
    )
    book_directive = """
directives:
- add_model:
    name: book
    file: package://manipulation/book.sdf
    default_free_body_pose:
        book:
            translation: [0.65, 0, 0]
"""
    scenario = AppendDirectives(scenario, data=book_directive)
    station = builder.AddSystem(
        MakeHardwareStation(scenario, meshcat, parser_prefinalize_callback=Setup)
    )
    plant = station.GetSubsystemByName("plant")
    scene_graph = station.GetSubsystemByName("scene_graph")

    controller = builder.AddSystem(TorqueController(plant, ctrl_fun, velocity))

    logger = builder.AddSystem(VectorLogSink(3))

    builder.Connect(
        controller.get_output_port(0), station.GetInputPort("iiwa.position")
    )
    builder.Connect(
        controller.get_output_port(1),
        station.GetInputPort("iiwa.torque"),
    )
    builder.Connect(controller.get_output_port(2), logger.get_input_port(0))

    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        controller.get_input_port(0),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.velocity_estimated"),
        controller.get_input_port(1),
    )

    diagram = builder.Build()

    # Initialize default positions for plant.
    simulator = Simulator(diagram)
    plant_context = plant.GetMyContextFromRoot(simulator.get_mutable_context())
    plant.SetPositions(
        plant_context,
        plant.GetModelInstanceByName("iiwa"),
        np.array([np.pi / 4, -np.pi / 3, np.pi / 3]),
    )

    station_context = station.GetMyContextFromRoot(simulator.get_mutable_context())
    station.GetInputPort("wsg.position").FixValue(station_context, [0.02])

    # if running_as_notebook:
    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(duration)
    meshcat.PublishRecording()
    # else:
    #     # TODO(terry-suh): we need to simulate this fully to grade student's answers, but CI won't be happy.
    #     simulator.AdvanceTo(duration)

    pose = plant.GetFreeBodyPose(plant_context, plant.GetBodyByName("book"))

    # Return these so that we can check the pose of each object.
    return logger.FindLog(simulator.get_context()), plant, plant_context

def compute_ctrl(p_pxz_now, v_pxz_now, x_des, f_des):
    """Compute control action given current position and velocities, as well as
    desired x-direction position p_des(t) / desired z-direction force f_des.
    You may set theta_des yourself, though we recommend regulating it to zero.
    Input:
      - p_pxz_now: np.array (dim 3), position of the finger. [thetay, px, pz]
      - v_pxz_now: np.array (dim 3), velocity of the finger. [wy, vx, vz]
      - x_des: float, desired position of the finger along the x-direction.
      - f_des: float, desired force on the book along the z-direction.
    Output:
      - u    : np.array (dim 3), spatial torques to send to the manipulator. [tau_y, fx, fz]
    """
    KP_x = 0.8
    KD_x = 0.5   
    KP_theta = 1.0  
    KD_theta = 1.5 

    theta_y, px, pz = p_pxz_now
    wy, vx, vz = v_pxz_now

    fx = KP_x * (x_des - px) + KD_x * vx
    tau_y = KP_theta * (-theta_y) + KD_theta * wy
    fz = -f_des
    u = np.array([tau_y, fx, fz])
    return u


def setup_manipulation_station(meshcat, control_func, ):
    builder = DiagramBuilder()
    scenario = LoadScenario(filename="models/simple.scenario.dmd.yaml")
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat, package_xmls=[os.getcwd() + "/package.xml"]))
    plant = station.GetSubsystemByName("plant")
    scene_graph = station.GetSubsystemByName("scene_graph")
    AddMultibodyTriad(plant.GetFrameByName("body"), scene_graph)

    return builder, plant, scene_graph, station

def BuildAndSimulateTrajectory(builder, station, q_traj, g_traj, meshcat, duration=0.01):
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
    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(duration)
    meshcat.PublishRecording()

    return simulator

def compute_ik(plant, context, end_effector_frame, desired_pose):
    ik = InverseKinematics(plant)
    ik.AddPositionConstraint(
        frameB=plant.world_frame(),
        p_BQ=desired_pose.translation(),
        frameA=plant.world_frame(),
        p_AQ_lower=desired_pose.translation(),
        p_AQ_upper=desired_pose.translation()
    )
    ik.AddOrientationConstraint(
        frameAbar=end_effector_frame,
        R_AbarA=desired_pose.rotation(),
        frameBbar=plant.world_frame(),
        R_BbarB=desired_pose.rotation(),
        theta_bound=0.01  # Allow small orientation errors
    )

    prog = ik.prog()
    result = prog.Solve()
    if result.is_success():
        return prog.GetSolution(ik.q())  # Joint positions
    else:
        raise RuntimeError("IK solution failed.")
    
def compute_handle_pose(cube_center_position, offset, rotation_angles, is_negative=False):
    """
    Compute the handle's position and rotation matrix for a given rotation and time.
    
    Parameters:
        cube_center_position (list): The cube's center position.
        offset (list): The offset for the handle's position.
        rotation_angles (tuple): Start and end angles (radians).
        t (float): Interpolation parameter (0 to 1).
        is_negative (bool): Whether to handle cases where t < 0.
    
    Returns:
        tuple: Position (numpy array) and rotation matrix.
    """
    angle_start, angle_end = rotation_angles
    p_Whandle = np.add(cube_center_position, offset)
    theta = angle_start + (angle_end - angle_start)

    if is_negative:
        p_Whandle += np.array([0, 0, 0.1]) if len(offset) == 3 else np.array([0, -0.1, 0])
        theta = angle_start

    return p_Whandle, theta

def InterpolatePoseRotate(rotation: str) -> RigidTransform:
    """
    Interpolates the pose for opening doors based on the rotation type and time.

    Parameters:
        t (float): Interpolation parameter (0 to 1). Negative values indicate pre-positioning.
        rotation (str): Rotation type ('U', 'U\'', 'F', 'F\'', 'R', 'R\'').

    Returns:
        RigidTransform: The interpolated rigid transform.
    """
    cube_center_position = [0.5, 0.5, 0.25]
    is_negative = t < 0

    # Define rotation configurations
    rotation_config = {
        'U': ([0.0, 0.0, 0.01], (np.pi / 2, 0)),
        'U\'': ([0.0, 0.0, 0.01], (np.pi / 2, np.pi)),
        'F': ([-0.01, 0.0, 0.0], (0, -np.pi / 2)),
        'F\'': ([-0.01, 0.0, 0.0], (0, -np.pi / 2)),
        'R': ([0.0, -0.01, 0.0], (np.pi, np.pi * 3 / 2)),
        'R\'': ([0.0, -0.01, 0.0], (np.pi, np.pi / 2)),
    }

    if rotation not in rotation_config:
        raise ValueError(f"Invalid rotation type: {rotation}")

    offset, rotation_angles = rotation_config[rotation]
    p_Whandle, theta = compute_handle_pose(cube_center_position, offset, rotation_angles, is_negative)

    # Determine roll-pitch-yaw order based on rotation
    if rotation in ['U', 'U\'']:
        R_Whandle = RollPitchYaw(np.pi / 2, 0, theta).ToRotationMatrix()
    elif rotation in ['F', 'F\'']:
        R_Whandle = RollPitchYaw(0, theta, np.pi / 2).ToRotationMatrix()
    else:  # 'R', 'R\''
        R_Whandle = RollPitchYaw(np.pi, theta, 0).ToRotationMatrix()

    X_Whandle = RigidTransform(R_Whandle, p_Whandle)

    # Add a gripper offset
    p_handleG = np.array([0.0, 0.114, 0.0])
    R_handleG = RollPitchYaw(0, np.pi, np.pi).ToRotationMatrix()
    X_handleG = RigidTransform(R_handleG, p_handleG)

    return X_Whandle.multiply(X_handleG)

    
if __name__ == "__main__":
    meshcat = StartMeshcat()
    builder, plant, scene_graph, station = setup_manipulation_station(meshcat)
    context = plant.CreateDefaultContext()
    gripper = plant.GetBodyByName("body")
    initial_pose = plant.EvalBodyPoseInWorld(context, gripper)

    desired_gripper_pose = 
    ik_solution = compute_ik(plant, context, gripper, )

    entry_traj_rotation = make_gripper_orientation_trajectory(initial_pose, rotation)
    entry_traj_translation = make_gripper_position_trajectory(initial_pose, rotation)

    entry_duration = 5.0
    grip_duration = 1.0
    rotate_duration = 5.0
    total_duration = entry_duration + grip_duration + rotate_duration
    interval_count = int(total_duration * 2 + 1)

    t_lst = np.linspace(0, total_duration, interval_count)
    pose_lst = []
    for t in t_lst:
        pose = InterpolatePose(t, rotation, entry_traj_rotation, entry_traj_translation, entry_duration, grip_duration, rotate_duration)
        AddMeshcatTriad(meshcat, path=str(t), X_PT = pose, opacity=0.02)
        pose_lst.append(pose)

    gripper_t_lst = np.array([0.0, 
                              entry_duration, 
                              entry_duration + grip_duration, 
                              entry_duration + grip_duration + rotate_duration])
    gripper_knots = np.array([0.08, 
                              0.08, 
                              0.0, 
                              0.0]).reshape(1, 4)
    g_traj = PiecewisePolynomial.FirstOrderHold(gripper_t_lst, gripper_knots)

    q_knots = np.array(create_q_knots(pose_lst))
    q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, q_knots[:, 0:7].T)
    simulator= BuildAndSimulateTrajectory(builder, station, q_traj, g_traj, meshcat, total_duration)