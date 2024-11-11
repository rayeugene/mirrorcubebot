import numpy as np
from pydrake.all import (
    ContactVisualizer,
    DiagramBuilder,
    Parser, 
    Simulator,
    StartMeshcat,
    ModelVisualizer,
    MeshcatVisualizer
)
from manipulation import ConfigureParser, FindResource, running_as_notebook
import os

def main():
    # Start the visualizer.
    meshcat = StartMeshcat()

    cube_file = "package://manipulation/rubiks_cube_2_by_2.sdf"
    # mirror_cube_file = "models/mirror_cube_2_by_2.sdf"

    meshcat.Delete()

    visualizer = ModelVisualizer(meshcat=meshcat)
    ConfigureParser(visualizer.parser())
    visualizer.parser().package_map().PopulateFromFolder(os.getcwd())
    visualizer.parser().AddModels(url="package://mirror_cube_bot/models/mirror_cube_2_by_2.sdf")
    # visualizer.parser().AddModelsFromUrl(cube_file)


    visualizer.Run(loop_once=False)

    meshcat.DeleteAddedControls()

if __name__ == "__main__":
    main()