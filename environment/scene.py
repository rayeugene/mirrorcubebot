import numpy as np
from pydrake.all import (
    DiagramBuilder, 
    LeafSystem,  
    MeshcatVisualizer,
    Simulator, 
    MeshcatVisualizerParams, 
    ConstantVectorSource, 
    Integrator, 
    TrajectorySource, 
    RigidTransform, 
    JacobianWrtVariable, 
    BasicVector,
    RotationMatrix,
    StartMeshcat
)
from manipulation.scenarios import MakeManipulationStation
from manipulation.meshcat_utils import AddMeshcatTriad
from manipulation import running_as_notebook

      #file: package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf
model_directives =  """
  directives:
  - add_model:
      name: iiwa0
      file:package://drake_models/iiwa_description/sdf/iiwa7_no_collision.sdf
      default_joint_positions:
          iiwa_joint_1: [-1.57]
          iiwa_joint_2: [0.1]
          iiwa_joint_3: [0]
          iiwa_joint_4: [-1.2]
          iiwa_joint_5: [0]
          iiwa_joint_6: [1.6]
          iiwa_joint_7: [0]
  - add_weld:
      parent: world
      child: iiwa0::iiwa_link_0
      X_PC:
          translation: [0, 0.7, 0]
          rotation: !Rpy { deg: [0, 0, -90] }
  - add_model:
      name: wsg0
      file: package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
  - add_weld:
      parent: iiwa0::iiwa_link_7
      child: wsg0::body
      X_PC:
          translation: [0, 0, 0.09]
          rotation: !Rpy { deg: [90, 0, 90] }
  - add_model:
      name: floor
      file: package://manipulation/floor.sdf
  - add_weld:
      parent: world
      child: floor::box
      X_PC:
          translation: [0, 0, -0.05]
  - add_model:
      name: cube
      file: models/cirror_cube_2_by_2.sdf
      default_free_body_pose:
          center:
              translation: [0, 0, 0.5]
  - add_model:
      name: stick
      file: package://manipulation/stick.sdf  # Placeholder; replace with an actual stick model.
  - add_weld:
      parent: floor::box
      child: stick::stick_base  # Weld the stick to the table.
      X_PC:
          translation: [0, 0, 0.25]  # Adjust the position of the stick base.
  - add_weld:
      parent: stick::stick_tip  # Weld the stick tip to a corner of the cube.
      child: cube::corner_0  # Replace 'corner_0' with the actual name of a corner frame in your cube model.
      X_PC:
          translation: [0, 0, 0]  # Ensure alignment with the cube corner.
"""

class IIWA_Painter():
    def __init__(self, traj=None):
        builder = DiagramBuilder()
        # set up the system of manipulation station
        self.station = builder.AddSystem(
            MakeManipulationStation(model_directives, package_xmls=["./package.xml"]))

        # builder.AddSystem(self.station)
        self.plant = self.station.GetSubsystemByName("plant")

        # optionally add trajectory source
        if traj is not None:
            traj_V_G = traj.MakeDerivative()
            V_G_source = builder.AddSystem(TrajectorySource(traj_V_G))
            self.controller = builder.AddSystem(
                PseudoInverseController(self.plant))
            builder.Connect(V_G_source.get_output_port(),
                            self.controller.GetInputPort("V_G"))

            self.integrator = builder.AddSystem(Integrator(7))
            builder.Connect(self.controller.get_output_port(),
                            self.integrator.get_input_port())
            builder.Connect(self.integrator.get_output_port(),
                            self.station.GetInputPort("iiwa0_position"))
            builder.Connect(
                self.station.GetOutputPort("iiwa0_position_measured"),
                self.controller.GetInputPort("iiwa0_position"))

        params = MeshcatVisualizerParams()
        params.delete_on_initialization_event = False
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            builder, self.station.GetOutputPort("query_object"), meshcat, params)

        wsg_position = builder.AddSystem(ConstantVectorSource([0.1]))
        builder.Connect(wsg_position.get_output_port(),
                        self.station.GetInputPort("wsg0_position"))

        self.diagram = builder.Build()
        self.gripper_frame = self.plant.GetFrameByName('body')
        self.world_frame = self.plant.world_frame()

        context = self.CreateDefaultContext()

        self.diagram.Publish(context)


    def visualize_frame(self, name, X_WF, length=0.15, radius=0.006):
        """
        visualize imaginary frame that are not attached to existing bodies
        
        Input: 
            name: the name of the frame (str)
            X_WF: a RigidTransform to from frame F to world.
        
        Frames whose names already exist will be overwritten by the new frame
        """
        AddMeshcatTriad(meshcat, "painter/" + name,
                        length=length, radius=radius, X_PT=X_WF)

    def CreateDefaultContext(self):
        context = self.diagram.CreateDefaultContext()
        plant_context = self.diagram.GetMutableSubsystemContext(
            self.plant, context)
        station_context = self.diagram.GetMutableSubsystemContext(
            self.station, context)

        # provide initial states
        q0 = np.array([ 1.40666193e-05,  1.56461165e-01, -3.82761069e-05,
                       -1.32296976e+00, -6.29097287e-06,  1.61181157e+00, -2.66900985e-05])
        # set the joint positions of the kuka arm
        iiwa = self.plant.GetModelInstanceByName("iiwa0")
        self.plant.SetPositions(plant_context, iiwa, q0)
        self.plant.SetVelocities(plant_context, iiwa, np.zeros(7))
        wsg = self.plant.GetModelInstanceByName("wsg0")
        self.plant.SetPositions(plant_context, wsg, [-0.025, 0.025])
        self.plant.SetVelocities(plant_context, wsg, [0, 0])        

        if hasattr(self, 'integrator'):
            self.integrator.set_integral_value(
                self.integrator.GetMyMutableContextFromRoot(context), q0)

        return context


    def get_X_WG(self, context=None):

        if not context:
            context = self.CreateDefaultContext()
        plant_context = self.plant.GetMyMutableContextFromRoot(context)
        X_WG = self.plant.CalcRelativeTransform(
                    plant_context,
                    frame_A=self.world_frame,
                    frame_B=self.gripper_frame)
        return X_WG

    def paint(self, sim_duration=20.0):
        context = self.CreateDefaultContext()
        simulator = Simulator(self.diagram, context)
        simulator.set_target_realtime_rate(1.0)

        duration = sim_duration if running_as_notebook else 0.01
        simulator.AdvanceTo(duration)

class PseudoInverseController(LeafSystem):
    """
    same controller seen in-class
    """
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._iiwa = plant.GetModelInstanceByName("iiwa0")
        self._G = plant.GetBodyByName("body").body_frame()
        self._W = plant.world_frame()

        self.V_G_port = self.DeclareVectorInputPort("V_G", BasicVector(6))
        self.q_port = self.DeclareVectorInputPort("iiwa0_position", BasicVector(7))
        self.DeclareVectorOutputPort("iiwa0_velocity", BasicVector(7),
                                     self.CalcOutput)
        self.iiwa_start = plant.GetJointByName("iiwa_joint_1").velocity_start()
        self.iiwa_end = plant.GetJointByName("iiwa_joint_7").velocity_start()

    def CalcOutput(self, context, output):
        V_G = self.V_G_port.Eval(context)
        q = self.q_port.Eval(context)
        self._plant.SetPositions(self._plant_context, self._iiwa, q)
        J_G = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context, JacobianWrtVariable.kV,
            self._G, [0,0,0], self._W, self._W)
        J_G = J_G[:,self.iiwa_start:self.iiwa_end+1] # Only iiwa terms.
        v = np.linalg.pinv(J_G).dot(V_G) #important
        output.SetFromVector(v)

def main():
    meshcat = StartMeshcat()


    # define center and radius
    radius = 0.05
    p0 = [0, 0.0, 0.06]
    R0 = RotationMatrix(np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]]).T)
    X_WorldCenter = RigidTransform(R0, p0)

    num_key_frames = 10
    """
    you may use different thetas as long as your trajectory starts
    from the Start Frame above and your rotation is positive
    in the world frame about +z axis
    thetas = np.linspace(0, 2*np.pi, num_key_frames)
    """
    thetas = np.linspace(0, 2*np.pi, num_key_frames)

    painter = IIWA_Painter()

if __name__ == "__main__":
    main()