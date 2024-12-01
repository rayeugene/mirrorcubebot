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
from pydrake.multibody import inverse_kinematics
from pydrake.trajectories import PiecewisePolynomial

from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation.scenarios import AddMultibodyTriad
from manipulation.station import LoadScenario, MakeHardwareStation, MakeMultibodyPlant


# Interpolate pose for opening doors.
def InterpolatePoseOpen(t):
    # Start by interpolating the yaw angle of the hinge.
    angle_start = np.pi / 2
    angle_end = np.pi
    theta = angle_start + (angle_end - angle_start) * t
    # Convert to position and rotation.
    p_Whandle = np.array([0.75, 0, 0.01])
    # Add some offset here to account for gripper yaw angle.
    R_Whandle = RollPitchYaw(np.pi/2, 0, theta).ToRotationMatrix()
    X_Whandle = RigidTransform(R_Whandle, p_Whandle)

    # Add a little offset to account for gripper.
    p_handleG = np.array([0.0, 0.114, 0.0])
    R_handleG = RollPitchYaw(0, np.pi, np.pi).ToRotationMatrix()
    X_handleG = RigidTransform(R_handleG, p_handleG)
    X_WG = X_Whandle.multiply(X_handleG)
    return X_WG

## Interpolate Pose for entry.
def make_gripper_orientation_trajectory(initial_pose):
    traj = PiecewiseQuaternionSlerp()
    traj.Append(0.0, initial_pose.rotation())
    traj.Append(5.0, InterpolatePoseOpen(0.0).rotation())
    return traj

def make_gripper_position_trajectory(initial_pose):
    traj = PiecewisePolynomial.FirstOrderHold(
        [0.0, 5.0],
        np.vstack(
            [
                [initial_pose.translation()],
                [InterpolatePoseOpen(0.0).translation()],
            ]
        ).T,
    )
    return traj

def InterpolatePoseEntry(t, entry_traj_rotation, entry_traj_translation):
    return RigidTransform(
        RotationMatrix(entry_traj_rotation.orientation(t)),
        entry_traj_translation.value(t),
    )

# Wrapper function for end-effector pose. Total time: 11 seconds.
def InterpolatePose(t, entry_traj_rotation, entry_traj_translation):
    if t < 5.0:
        # Duration of entry motion is set to 5 seconds.
        return InterpolatePoseEntry(t, entry_traj_rotation, entry_traj_translation)
    elif (t >= 5.0) and (t < 6.0):
        # Wait for a second to grip the handle.
        return InterpolatePoseEntry(5.0,entry_traj_rotation, entry_traj_translation)
    else:
        # Duration of the open motion is set to 5 seconds.
        return InterpolatePoseOpen((t - 6.0) / 5.0)


def CreateIiwaControllerPlant():
    """creates plant that includes only the robot and gripper, used for controllers."""
    scenario = LoadScenario(filename="models/simple.scenario.dmd.yaml")
    plant_robot = MakeMultibodyPlant(
        scenario=scenario, model_instance_names=["iiwa", "wsg"]
    )

    link_frame_indices = []
    for i in range(8):
        link_frame_indices.append(
            plant_robot.GetFrameByName("iiwa_link_" + str(i)).index()
        )

    return plant_robot, link_frame_indices

def setup_manipulation_station(meshcat):
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

def create_q_knots(pose_lst):
    """Convert end-effector pose list to joint position list using series of
    InverseKinematics problems. Note that q is 9-dimensional because the last 2 dimensions
    contain gripper joints, but these should not matter to the constraints.
    @param: pose_lst (python list): post_lst[i] contains keyframe X_WG at index i.
    @return: q_knots (python_list): q_knots[i] contains IK solution that will give f(q_knots[i]) \approx pose_lst[i].
    """
    q_knots = []
    plant, _ = CreateIiwaControllerPlant()
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
        positional_tolerance = 0.001
        p_WG_lower = np.array([p_WG[0]-positional_tolerance, p_WG[1]-positional_tolerance, p_WG[2]-positional_tolerance])
        p_WG_upper = np.array([p_WG[0]+positional_tolerance, p_WG[1]+positional_tolerance, p_WG[2]+positional_tolerance])
        AddPositionConstraint(ik, p_WG_lower, p_WG_upper)

        # Orientation constraint to ensure pitch alignment towards y-axis
        theta_bound = 0.05 
        AddOrientationConstraint(ik, R_WG, bounds=theta_bound)

        # Add joint-centering cost to maintain natural joint positions
        prog.AddQuadraticErrorCost(np.eye(len(q_variables)), q_nominal, q_variables)

        # Set initial guess
        if i == 0:
            prog.SetInitialGuess(q_variables, q_nominal)
        else:
            prog.SetInitialGuess(q_variables, q_knots[-1])  

        result = Solve(prog)
        assert result.is_success()
        q_knots.append(result.GetSolution(q_variables))

    return q_knots

def main():
    meshcat = StartMeshcat()

    builder, plant, scene_graph, station = setup_manipulation_station(meshcat)
    context = plant.CreateDefaultContext()
    gripper = plant.GetBodyByName("body")
    initial_pose = plant.EvalBodyPoseInWorld(context, gripper)

    entry_traj_rotation = make_gripper_orientation_trajectory(initial_pose)
    entry_traj_translation = make_gripper_position_trajectory(initial_pose)

    t_lst = np.linspace(0, 11, 30)
    pose_lst = []
    for t in t_lst:
        AddMeshcatTriad(meshcat, path=str(t), X_PT=InterpolatePose(t, entry_traj_rotation, entry_traj_translation), opacity=0.02)
        pose_lst.append(InterpolatePose(t, entry_traj_rotation, entry_traj_translation))

    gripper_t_lst = np.array([0.0, 5.0, 6.0, 11.0])
    gripper_knots = np.array([0.08, 0.08, 0.0, 0.0]).reshape(1, 4)
    g_traj = PiecewisePolynomial.FirstOrderHold(gripper_t_lst, gripper_knots)

    q_knots = np.array(create_q_knots(pose_lst))
    q_traj = PiecewisePolynomial.CubicShapePreserving(t_lst, q_knots[:, 0:7].T)
    simulator= BuildAndSimulateTrajectory(builder, station, q_traj, g_traj, meshcat, 11.0)


if __name__ == "__main__":
    main()