import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from rubiks_cube.cube_generator import CubeGenerator
from rubiks_cube.static_corner_generator import BlockGenerator
import numpy as np

cube_lengths = [0.025, 0.03, 0.035, 0.04, 0.045]
cubie_ratios = [0.005, 0.01, 0.015]
friction = 0.6
spacing = 0.0001

def cube_generation():
    for i, cube_length in enumerate(cube_lengths):
        for j, cubie_ratio in enumerate(cubie_ratios):
            #print(np.array([cube_length - cubie_ratio, cube_length, cube_length + cubie_ratio]))
            file_name = "models/cube_length_cubie_ratio/mirror_cube_l_" + str(i) + "_r_" + str(j) + ".sdf"
            CubeGenerator.generate_mirror_2_by_2(
                edge_length = cube_length * 2, 
                cube_center = np.array([cube_length - cubie_ratio, cube_length, cube_length + cubie_ratio]), 
                mass_density = 390, 
                file_out = file_name,
                layer_spacing = spacing, 
                friction_coeff=np.array([friction, friction])
            ) 


edge_length = 0.01
cube_center_position = np.array([0.50, 0.50, 0.25])
cubie_offset = np.array([0.04, 0.03, -0.04])
for i in range(3):
    block_offset = [- edge_length/2 * np.sign(x) for x in cubie_offset]
    block_offset[i] = -block_offset[i]
    position = np.add(np.add(cube_center_position, cubie_offset), np.array(block_offset))

    BlockGenerator.generate_static_corner_block(
        edge_length=edge_length, 
        corner_position= position,
        mass_density= 390,
        solid_color={"name": "white", "rgba": np.array([0, 0, 0, 1])},
        file_out = "models/blocks/static_corner_block_" + str(i+1) + ".sdf" 
    )

def block_generation():
    edge_length = 0.01
    cube_center_position = np.array([0.50, 0.50, 0.25])

    for i, cube_length in enumerate(cube_lengths):
        for j, cubie_ratio in enumerate(cubie_ratios):

            cubie_offset = np.array([cube_length + cubie_ratio, cube_length, -cube_length - cubie_ratio])
            for k in range(3):
                block_offset = [- edge_length/2 * np.sign(x) for x in cubie_offset]
                block_offset[k] = -block_offset[k]
                position = np.add(np.add(cube_center_position, cubie_offset), np.array(block_offset))

                BlockGenerator.generate_static_corner_block(
                    edge_length= edge_length, 
                    corner_position= position,
                    mass_density= 390,
                    solid_color={"name": "white", "rgba": np.array([0, 0, 0, 1])},
                    file_out = "models/cube_length_cubie_ratio/static_corner_block_l_" + str(i) + "_r_" + str(j) + '_' + str(k+1) + ".sdf" 
                )

def scenario_generation():
    for i, cube_length in enumerate(cube_lengths):
        for j, cubie_ratio in enumerate(cubie_ratios):
            original_sdf_file_name = "models/cubes/mirror_cube_2_by_2.sdf"
            sdf_file_name = "models/cube_length_cubie_ratio/mirror_cube_l_" + str(i) + "_r_" + str(j) + ".sdf"
            
            with open("models/urf.rotation.scenario.dmd.yaml", "r") as file:
                yaml_content = file.read()
            yaml_content = yaml_content.replace (original_sdf_file_name, sdf_file_name)

            for k in range(3):
                original_block_sdf_file_name = "models/blocks/static_corner_block_" + str(k+1) + ".sdf"
                block_sdf_file_name = "models/cube_length_cubie_ratio/static_corner_block_l_" + str(i) + "_r_" + str(j) + '_' + str(k+1) + ".sdf" 
                yaml_content = yaml_content.replace (original_block_sdf_file_name, block_sdf_file_name)

            yaml_file_name = "models/cube_length_cubie_ratio/l_" +str(i) + "_r_" + str(j) + "scenario.dmd.yaml"
            with open(yaml_file_name, "w") as file:
                file.write(yaml_content)

def main ():
    cube_generation()
    block_generation()
    scenario_generation()
    
if __name__ == "__main__":
    main()