import os
import numpy as np
from pydrake.all import (
    ContactVisualizer,
    DiagramBuilder,
    Parser,
    Simulator,
    StartMeshcat,
    ModelVisualizer,
    MeshcatVisualizer,
    MultibodyPlant,
    SceneGraph,
    AddMultibodyPlantSceneGraph    
)
from manipulation import ConfigureParser, FindResource
from manipulation.utils import RenderDiagram
import supercube
from cube_scrambler import CubeScrambler


def main():
    meshcat = StartMeshcat()

    mirror_cube_file = "package://mirror_cube_bot/models/cubes/mirror_cube_2_by_2.sdf"

    # Create a DiagramBuilder
    builder = DiagramBuilder()

    # Create the MultibodyPlant and SceneGraph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.001)
    parser = Parser(plant)
    ConfigureParser(parser)
    parser.package_map().PopulateFromFolder(os.getcwd())
    parser.AddModelsFromUrl(mirror_cube_file)

    # Finalize the plant
    plant.Finalize()
    context = plant.CreateDefaultContext()

    pocket_cube = supercube.PocketCube()
    scramble = 'F'
    pocket_cube.move(scramble)
    # scramble = pocket_cube.scramble().split()

    scrambler = CubeScrambler()
    robot_scramble_moves = scrambler.convert_sequence_to_robot_space(scramble)
    scrambler.apply_sequence(robot_scramble_moves)
    cube_joint_init_state = scrambler.get_joint_rpys()
    print(cube_joint_init_state)

    # cube_joint_init_state = np.zeros((2,2,2,3))

    # moved_idx = [(0,1,0),(0,1,1),(1,1,0),(1,1,1)]
    # for idx in moved_idx:
    #     cube_joint_init_state = 

    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(builder, plant, meshcat)

    # builder.Connect(
    #     scene_graph.get_query_output_port(),
    #     plant.get_geometry_query_input_port()
    # )

    diagram = builder.Build()

    RenderDiagram(diagram)

    plant_context = plant.GetMyMutableContextFromRoot(context)
    for x in range(2):
        for y in range(2):
            for z in range(2):
                joint_name = f"ball_{x}_{y}_{z}"
                joint = plant.GetJointByName(joint_name)
                joint.set_angles(plant_context, cube_joint_init_state[x,y,z,:])

    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)

    meshcat.StartRecording()
    simulator.AdvanceTo(5.0)
    meshcat.StopRecording()
    meshcat.PublishRecording()


if __name__ == "__main__":
    main()
