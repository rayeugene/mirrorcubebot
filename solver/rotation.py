import numpy as np
from  pydrake.all import RollPitchYaw

cubies_to_numbers = {
    '000': [22,12,15],
    '001': [4,10,13],
    '010': [21,8,11],
    '011': [3,6,9],
    '100': [24,16,19],
    '101': [2,14,17],
    '110': [23,7,20],
    '111': [1,5,18],
}

def get_rotation(state, rotation, cubie, previous_rotation):
    number = cubies_to_numbers[cubie][0]

    for x in range(6):
        for y in range(2):
            for z in range(2):
                if state[x][y][z] == number:
                    original_number = 4 * x + 2 * y + z + 1
                    break
    
    for original_cubie, numbers in cubies_to_numbers.items():
        if original_number in numbers:
            break
    rotation_face = rotation[0]
    orientation = RollPitchYaw(0,0,0)
    if rotation_face == 'U':
        if original_cubie[2] == '1':
            angle = -np.pi/2
            if rotation_face != rotation:
                angle = -angle
            orientation = RollPitchYaw(0, 0, angle)
    elif rotation_face == 'F':
        if original_cubie[0] == '0':
            angle = np.pi/2
            if rotation_face != rotation:
                angle = -angle
            orientation = RollPitchYaw(angle, 0, 0)
    elif rotation_face == 'R':
        if original_cubie[1] == '0':
            angle = np.pi/2
            if rotation_face != rotation:
                angle = -angle
            orientation = RollPitchYaw(0, angle, 0)
    
    return previous_rotation @ orientation.ToRotationMatrix()
    
    
    


