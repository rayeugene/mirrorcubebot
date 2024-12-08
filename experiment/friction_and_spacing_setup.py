import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from rubiks_cube.cube_generator import CubeGenerator
import numpy as np

frictions = [1, 0.1, 0.01, 0.001, 0.0001]
spacings = [0.001, 0.0001, 0.00001]

def cube_generation():
    for i, friction in enumerate(frictions):
        for j, spacing in enumerate(spacings):
            file_name = "models/friction_and_spacing/mirror_cube_f_" + str(i) + "_s_" + str(j) + ".sdf"
            CubeGenerator.generate_mirror_2_by_2(
                edge_length = 0.06, 
                cube_center = np.array([0.02, 0.03, 0.04]), 
                mass_density = 390, 
                file_out = file_name,
                layer_spacing = spacing, 
                friction_coeff=np.array([friction, friction])
            ) 
def scenario_generation():
    for i, friction in enumerate(frictions):
        for j, spacing in enumerate(spacings):
            original_sdf_file_name = "models/cubes/mirror_cube_2_by_2.sdf"
            sdf_file_name = "models/friction_and_spacing/mirror_cube_f_" + str(i) + "_s_" + str(j) + ".sdf"
            yaml_file_name = "models/friction_and_spacing/f_" +str(i) + "_s_" + str(j) + "scenario.dmd.yaml"
            with open("models/urf.rotation.scenario.dmd.yaml", "r") as file:
                yaml_content = file.read()
            yaml_content = yaml_content.replace (original_sdf_file_name, sdf_file_name)
            with open(yaml_file_name, "w") as file:
                file.write(yaml_content)

def main ():
    cube_generation()
    scenario_generation()
    
if __name__ == "__main__":
    main()