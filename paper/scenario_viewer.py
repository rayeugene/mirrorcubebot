import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from pydrake.all import (
    ContactVisualizer,
    DiagramBuilder,
    Parser,
    Simulator,
    StartMeshcat,
    ModelVisualizer,
    MeshcatVisualizer,
    AddMultibodyPlantSceneGraph,
    LoadModelDirectives,
    ProcessModelDirectives
)
from manipulation.station import LoadScenario, MakeHardwareStation, MakeMultibodyPlant
from manipulation import ConfigureParser, FindResource, running_as_notebook


def main():
    scenario_file = "paper/models/cube_env.scenario.yaml"
    # scenario_file = "paper/models/mirror_vs_normal_env.scenario.yaml"

    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    scenario = LoadScenario(filename=scenario_file)
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat, package_xmls=[os.getcwd() + "/package.xml"]))

    diagram = builder.Build()
    simulator = Simulator(diagram)

    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(2)
    meshcat.PublishRecording()


if __name__ == "__main__":
    main()
