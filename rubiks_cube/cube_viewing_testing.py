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

def main():
    # Start the visualizer.
    meshcat = StartMeshcat()

    cube_file = "package://manipulation/rubiks_cube_2_by_2.sdf"

    meshcat.Delete()

    visualizer = ModelVisualizer(meshcat=meshcat)
    ConfigureParser(visualizer.parser())
    visualizer.parser().AddModelsFromUrl(cube_file)

    visualizer.Run(loop_once=False)

    meshcat.DeleteAddedControls()

if __name__ == "__main__":
    main()