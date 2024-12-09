import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from rubiks_cube.cube_generator import CubeGenerator
from rubiks_cube.static_corner_generator import BlockGenerator
import numpy as np

cubie_ratios = [0.005, 0.010, 0.015]
cube_length = 0.03
friction = 0.6
spacing = 0.0001


def cube_generation():
    for i, cubie_ratio in enumerate(cubie_ratios):
        #print(np.array([cube_length - cubie_ratio, cube_length, cube_length + cubie_ratio]))
        file_name = "models/grasping_position_cm_offset/mirror_cube_r_" + str(i) + ".sdf"
        CubeGenerator.generate_mirror_2_by_2(
            edge_length = cube_length * 2, 
            cube_center = np.array([cube_length - cubie_ratio, cube_length, cube_length + cubie_ratio]), 
            mass_density = 390, 
            file_out = file_name,
            layer_spacing = spacing, 
            friction_coeff=np.array([friction, friction])
        ) 

def scenario_generation():
    for i, cube_length in enumerate(cubie_ratios):
        original_sdf_file_name = "models/cubes/mirror_cube_2_by_2.sdf"
        sdf_file_name = "models/grasping_position_cm_offset/mirror_cube_r_" + str(i) + ".sdf"
        
        with open("models/urf.rotation.scenario.dmd.yaml", "r") as file:
            yaml_content = file.read()
        yaml_content = yaml_content.replace (original_sdf_file_name, sdf_file_name)

        yaml_file_name ="models/grasping_position_cm_offset/mirror_cube_r_" + str(i) + ".scenario.dmd.yaml"
        with open(yaml_file_name, "w") as file:
            file.write(yaml_content)

def main ():
    cube_generation()
    scenario_generation()
    
if __name__ == "__main__":
    main()