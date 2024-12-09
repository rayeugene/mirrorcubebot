import numpy as np
from pydrake.all import (
    DiagramBuilder,
    DiscreteContactApproximation,
    JacobianWrtVariable,
    LeafSystem,
    RollPitchYaw,
    Simulator,
    StartMeshcat,
    VectorLogSink,
    RigidTransform,
    PiecewiseQuaternionSlerp,
    RotationMatrix,
    PiecewisePolynomial
)

from manipulation import running_as_notebook
from manipulation.station import AppendDirectives, LoadScenario, MakeHardwareStation
from manipulation.utils import FindResource

class CubeGraspControl():
    def __init__(self):
        pass

    def compute_handle_pose(self, cube_center_position, offset, rotation_angles, t, is_negative=False):
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
        theta = angle_start + (angle_end - angle_start) * t

        if is_negative:
            p_Whandle += np.array([0, 0, 0.1]) if len(offset) == 3 else np.array([0, -0.1, 0])
            theta = angle_start

        return p_Whandle, theta

    def InterpolatePoseRotate(self, t: float, rotation: str) -> RigidTransform:
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
        p_Whandle, theta = self.compute_handle_pose(cube_center_position, offset, rotation_angles, t, is_negative)

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

    ## Interpolate Pose for entry.
    def make_gripper_orientation_trajectory(self, initial_pose, rotation):
        traj = PiecewiseQuaternionSlerp()
        traj.Append(0.0, initial_pose.rotation())
        traj.Append(0.8, self.InterpolatePoseRotate(-1.0, rotation).rotation())
        traj.Append(1.0, self.InterpolatePoseRotate(0.0, rotation).rotation())
        return traj

    def make_gripper_position_trajectory(self, initial_pose, rotation):
        traj = PiecewisePolynomial.FirstOrderHold(
            [0.0, 0.8, 1.0],
            np.vstack(
                [
                    [initial_pose.translation()],
                    [self.InterpolatePoseRotate(-1.0, rotation).translation()],
                    [self.InterpolatePoseRotate(0.0, rotation).translation()],
                ]
            ).T,
        )
        return traj

    def InterpolatePoseEntry(self, t, entry_traj_rotation, entry_traj_translation):
        return RigidTransform(
            RotationMatrix(entry_traj_rotation.orientation(t)),
            entry_traj_translation.value(t),
        )

    def InterpolatePose(self, t, rotation, entry_traj_rotation, entry_traj_translation, entry_duration, grip_duration, rotate_duration):
        if t < entry_duration:
            return self.InterpolatePoseEntry(t / entry_duration if entry_duration != 0 else 0.0, 
                                        entry_traj_rotation, 
                                        entry_traj_translation)
        elif t < entry_duration + grip_duration:
            return self.InterpolatePoseEntry(1.0, 
                                        entry_traj_rotation, 
                                        entry_traj_translation)
        elif t < entry_duration + grip_duration + rotate_duration:
            return self.InterpolatePoseRotate((t - (entry_duration + grip_duration)) / rotate_duration, rotation)
        else: 
            return self.InterpolatePoseRotate(1.0, rotation)

class TorqueController(LeafSystem):
    """Wrapper System for Commanding Pure Torques to planar iiwa.
    @param plant MultibodyPlant of the simulated plant.
    @param ctrl_fun function object to implement torque control law.
    """

    def __init__(self, plant, ctrl_fun):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._iiwa = plant.GetModelInstanceByName("iiwa")
        self._G = plant.GetBodyByName("body").body_frame()
        self._W = plant.world_frame()
        self._ctrl_fun = ctrl_fun
        self._joint_indices = [
            plant.GetJointByName("iiwa_joint_7").position_start()            
        ]

        # self.cube_body = self._plant.GetBodyByName("mirror_cube_2x2").body_frame()

        self.DeclareVectorInputPort("iiwa_position_measured", 7)
        self.DeclareVectorInputPort("iiwa_velocity_measured", 7)

        # If we want, we can add this in to do closed-loop force control on z.
        # self.DeclareVectorInputPort("iiwa_torque_external", 3)

        self.DeclareVectorOutputPort(
            "iiwa_position_command", 7, self.CalcPositionOutput
        )
        self.DeclareVectorOutputPort("iiwa_torque_cmd", 7, self.CalcTorqueOutput)
        # Compute foward kinematics so we can log the wsg position for grading.
        self.DeclareVectorOutputPort("wsg_position", 7, self.CalcWsgPositionOutput)

    def CalcPositionOutput(self, context, output):
        """Set q_d = q_now. This ensures the iiwa goes into pure torque mode in sim by setting the position control torques in InverseDynamicsController to zero.
        NOTE(terry-suh): Do not use this method on hardware or deploy this notebook on hardware.
        We can only simulate pure torque control mode for iiwa on sim.
        """
        q_now = self.get_input_port(0).Eval(context)
        output.SetFromVector(q_now)

    def CalcTorqueOutput(self, context, output):
        # Hard-coded position and force profiles. Can be connected from Trajectory class.
        if context.get_time() < 6.0:
            px_des = 0
        else:
            px_des = 90

        # Read inputs
        q_now = self.get_input_port(0).Eval(context)
        v_now = self.get_input_port(1).Eval(context)
        # tau_now = self.get_input_port(2).Eval(context)

        self._plant.SetPositions(self._plant_context, self._iiwa, q_now)

        # 1. Convert joint space quantities to Cartesian quantities.
        X_now = self._plant.CalcRelativeTransform(self._plant_context, self._W, self._G)

        rpy_now = RollPitchYaw(X_now.rotation()).vector()
        p_xyz_now = X_now.translation()

        J_G = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context,
            JacobianWrtVariable.kQDot,
            self._G,
            [0, 0, 0],
            self._W,
            self._W,
        )

        # Only select relevant terms. We end up with J_G of shape (3,3).
        # Rows correspond to (pitch, x, z).
        # Columns correspond to (q0, q1, q2).
        J_G = J_G[np.ix_([6], self._joint_indices)]
        v_pxz_now = J_G.dot(v_now)

        # p_pxz_now = np.array([rpy_now[2], p_xyz_now[0], p_xyz_now[2]])

        # 2. Apply ctrl_fun
        p_G_rot_deg = rpy_now[2]
        F_pxz = self._ctrl_fun(p_G_rot_deg, v_pxz_now, px_des)

        # 3. Convert back to joint coordinates'
        tau_cmd = np.zeros(7)
        tau_cmd[6] = J_G.T.dot(F_pxz)
        output.SetFromVector(tau_cmd)

    def CalcWsgPositionOutput(self, context, output):
        """
        Compute Forward kinematics. Needed to log the position trajectory for grading.  TODO(russt): Could use MultibodyPlant's body_poses output port for this.
        """
        q_now = self.get_input_port(0).Eval(context)
        self._plant.SetPositions(self._plant_context, self._iiwa, q_now)
        X_now = self._plant.CalcRelativeTransform(self._plant_context, self._W, self._G)

        rpy_now = RollPitchYaw(X_now.rotation()).vector()
        p_xyz_now = X_now.translation()
        p_pxz_now = np.array([rpy_now[1], p_xyz_now[0], p_xyz_now[2]])

        output.SetFromVector(p_pxz_now)

# meshcat = StartMeshcat()

# def Setup(parser):
#     parser.plant().set_discrete_contact_approximation(
#         DiscreteContactApproximation.kLagged
#     )


# def BuildAndSimulate(ctrl_fun, velocity, duration):
#     builder = DiagramBuilder()

#     scenario = LoadScenario(
#         filename=FindResource("models/planar_manipulation_station.scenario.yaml")
#     )
#     book_directive = """
# directives:
# - add_model:
#     name: book
#     file: package://manipulation/book.sdf
#     default_free_body_pose:
#         book:
#             translation: [0.65, 0, 0]
# """
#     scenario = AppendDirectives(scenario, data=book_directive)
#     station = builder.AddSystem(
#         MakeHardwareStation(scenario, meshcat, parser_prefinalize_callback=Setup)
#     )
#     plant = station.GetSubsystemByName("plant")
#     scene_graph = station.GetSubsystemByName("scene_graph")

#     controller = builder.AddSystem(TorqueController(plant, ctrl_fun))

#     logger = builder.AddSystem(VectorLogSink(3))

#     builder.Connect(
#         controller.get_output_port(0), station.GetInputPort("iiwa.position")
#     )
#     builder.Connect(
#         controller.get_output_port(1),
#         station.GetInputPort("iiwa.torque"),
#     )
#     builder.Connect(controller.get_output_port(2), logger.get_input_port(0))

#     builder.Connect(
#         station.GetOutputPort("iiwa.position_measured"),
#         controller.get_input_port(0),
#     )
#     builder.Connect(
#         station.GetOutputPort("iiwa.velocity_estimated"),
#         controller.get_input_port(1),
#     )

#     diagram = builder.Build()

#     # Initialize default positions for plant.
#     simulator = Simulator(diagram)
#     plant_context = plant.GetMyContextFromRoot(simulator.get_mutable_context())
#     # plant.SetPositions(
#     #     plant_context,
#     #     plant.GetModelInstanceByName("iiwa"),
#     #     np.array([np.pi / 4, -np.pi / 3, np.pi / 3]),
#     # )

#     station_context = station.GetMyContextFromRoot(simulator.get_mutable_context())
#     station.GetInputPort("wsg.position").FixValue(station_context, [0.02])

#     # if running_as_notebook:
#     meshcat.StartRecording(set_visualizations_while_recording=False)
#     simulator.AdvanceTo(duration)
#     meshcat.PublishRecording()
#     # else:
#     #     # TODO(terry-suh): we need to simulate this fully to grade student's answers, but CI won't be happy.
#     # simulator.AdvanceTo(duration)

#     pose = plant.GetFreeBodyPose(plant_context, plant.GetBodyByName("book"))

#     # Return these so that we can check the pose of each object.
#     return logger.FindLog(simulator.get_context()), plant, plant_context

# def compute_ctrl(p_pxz_now, v_pxz_now, x_des):
#     """Compute control action given current position and velocities, as well as
#     desired x-direction position p_des(t) / desired z-direction force f_des.
#     You may set theta_des yourself, though we recommend regulating it to zero.
#     Input:
#       - p_pxz_now: np.array (dim 1), position of the effector. [theta]
#       - v_pxz_now: np.array (dim 1), velocity of the effector. [w]
#     Output:
#       - u    : np.array (dim 1), spatial torques to send to the manipulator. [tau]
#     """
#     KP_theta = 1.0  
#     KD_theta = 1.5 

#     theta_y = p_pxz_now
#     wy = v_pxz_now

#     tau_y = KP_theta * (x_des-theta_y) + KD_theta * wy
#     u = np.array([tau_y])
#     return u


# # NOTE: We recommend the following values, though you may try different velocities and durations as well!
# velocity = -0.125  # p_des = 0.65 + velocity * max\{time - 2.0, 0\}
# duration = 6.5  # duration to simulate. We check the book pose at the end of duration. set to 5~10.
# log, plant, plant_context = BuildAndSimulate(compute_ctrl, velocity, duration)