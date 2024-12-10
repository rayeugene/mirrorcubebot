import sys
import os

# Add the project root directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np
from rubiks_cube.cube_generator import CubeGenerator as CG

if __name__ == "__main__":
    # # to show difference in size
    # CG.generate_mirror_2_by_2(
    #     edge_length = 0.12, 
    #     cube_center = np.array([0.04, 0.06, 0.08]), 
    #     mass_density = 390, 
    #     file_out = "paper/models/mirror_big.sdf",
    #     layer_spacing = 0.0004, 
    #     friction_coeff=np.array([0,0])
    # )   
    # CG.generate_mirror_2_by_2(
    #     edge_length = 0.06, 
    #     cube_center = np.array([0.02, 0.03, 0.04]), 
    #     mass_density = 390, 
    #     file_out = "paper/models/mirror_small.sdf",
    #     layer_spacing = 0.0004, 
    #     friction_coeff=np.array([0,0])
    # )   

    # # to show difference in eccentricity
    # CG.generate_mirror_2_by_2(
    #     edge_length = 0.06, 
    #     cube_center = np.array([0.04, 0.03, 0.05]), 
    #     mass_density = 390, 
    #     file_out = "paper/models/mirror_1.sdf",
    #     layer_spacing = 0.0004, 
    #     friction_coeff=np.array([0,0])
    # )   
    # CG.generate_mirror_2_by_2(
    #     edge_length = 0.06, 
    #     cube_center = np.array([0.02, 0.03, 0.04]), 
    #     mass_density = 390, 
    #     file_out = "paper/models/mirror_2.sdf",
    #     layer_spacing = 0.0004, 
    #     friction_coeff=np.array([0,0])
    # )   

    # # to show difference in layer spacing
    # CG.generate_mirror_2_by_2(
    #     edge_length = 0.06, 
    #     cube_center = np.array([0.02, 0.03, 0.04]), 
    #     mass_density = 390, 
    #     file_out = "paper/models/mirror_big_space.sdf",
    #     layer_spacing = 0.005, 
    #     friction_coeff=np.array([0,0])
    # )   
    # CG.generate_mirror_2_by_2(
    #     edge_length = 0.06, 
    #     cube_center = np.array([0.02, 0.03, 0.04]), 
    #     mass_density = 390, 
    #     file_out = "paper/models/mirror_normal_space.sdf",
    #     layer_spacing = 0.0004, 
    #     friction_coeff=np.array([0,0])
    # )   


    #generate_normal_cube
    CG.generate_mirror_2_by_2(
        edge_length = 0.06, 
        cube_center = np.array([0.03, 0.03, 0.03]), 
        mass_density = 390, 
        file_out = "paper/models/pocket_cube.sdf",
        layer_spacing = 0.0004, 
        friction_coeff=np.array([0,0])
    )   