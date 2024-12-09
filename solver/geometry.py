same_cubies = [
    [1,5,18],
    [2,14,17],
    [3,6,9],
    [4,10,13],
    [21,8,11],
    [22,12,15],
    [23,7,20],
    [24,16,19]
]
def get_volume(number, state, heights):
    for cubie in same_cubies:
        if number in cubie:
            break
    result  = 1
    for number in cubie:
        number -= 1
        x = number // 4
        y = (number - 4 * x) // 2
        z = number - 4 * x - 2 * y
        result *= heights[state[x][y][z]]
    return result

def get_grip_position(state, heights, rotation):
    rotation_face = rotation[0]
    if rotation_face == 'U':
        top_left_volume = get_volume(1, state, heights)
        top_right_volume = get_volume(2, state, heights)
        bottom_left_volume = get_volume(3, state, heights)
        bottom_right_volume = get_volume(4, state, heights)
    elif rotation_face  == 'F':
        top_left_volume = get_volume(11, state, heights)
        top_right_volume =get_volume(9, state, heights)
        bottom_left_volume = get_volume(12, state, heights)
        bottom_right_volume = get_volume(10, state, heights)
    elif rotation_face == 'R':
        top_left_volume = get_volume(13, state, heights)
        top_right_volume = get_volume(14, state, heights)
        bottom_left_volume = get_volume(15, state, heights)
        bottom_right_volume = get_volume(16, state, heights)
    else :
        raise ValueError
    
    if rotation_face == 'U':
        top_left = heights[state[1][0][0]]
        top_right = heights[state[3][0][1]]
        bottom_left = heights[state[1][0][1]]
        bottom_right = heights[state[3][0][0]]
    elif rotation_face  == 'F':
        top_left = heights[state[5][0][0]]
        top_right = heights[state[0][1][0]]
        bottom_left = heights[state[5][0][1]]
        bottom_right = heights[state[0][1][1]]
    elif rotation_face == 'R':
        top_left = heights[state[2][0][1]]
        top_right = heights[state[4][0][0]]
        bottom_left = heights[state[2][1][1]]
        bottom_right = heights[state[4][1][0]]
    else :
        raise ValueError
    
    top = top_left_volume + top_right_volume
    bottom = bottom_left_volume + bottom_right_volume

    if top >= bottom : # grip bottom
        grip_center = ((bottom_right - bottom_left)/2, 0.01)
    else: # grip top
        grip_center = ((top_right - top_left)/2, -0.01)
    return grip_center

def assign_heights(lengths):
    # lengths : the length of Up, Left, Front, Right, Back, and Down
    heights = {}
    for i in range(6):
        for j in range(4):
            length = lengths[i]
            heights[4 * i + j + 1] = length
    return heights

def get_center_of_mass(state, heights, rotation):
    horizontal_offset, vertical_offset = 0, 0 # x, y 

    rotation_face = rotation[0]
    if rotation_face == 'U':
        top_left_volume = get_volume(1, state, heights)
        top_right_volume = get_volume(2, state, heights)
        bottom_left_volume = get_volume(3, state, heights)
        bottom_right_volume = get_volume(4, state, heights)
    elif rotation_face  == 'F':
        top_left_volume = get_volume(11, state, heights)
        top_right_volume =get_volume(9, state, heights)
        bottom_left_volume = get_volume(12, state, heights)
        bottom_right_volume = get_volume(10, state, heights)
    elif rotation_face == 'R':
        top_left_volume = get_volume(13, state, heights)
        top_right_volume = get_volume(14, state, heights)
        bottom_left_volume = get_volume(15, state, heights)
        bottom_right_volume = get_volume(16, state, heights)
    else :
        raise ValueError
    total_volume = top_left_volume + top_right_volume + bottom_right_volume + bottom_left_volume
    
    if rotation_face == 'U':
        top_left_horizontal_offset = heights[state[1][0][0]]
        top_left_vertical_offset = heights[state[4][0][1]]
        top_right_horizontal_offset = heights[state[3][0][1]]
        top_right_vertical_offset = heights[state[4][0][0]]
        bottom_left_horizontal_offset = heights[state[1][0][1]]
        bottom_left_vertical_offset = heights[state[2][0][0]]
        bottom_right_horizontal_offset = heights[state[3][0][0]]
        bottom_right_vertical_offset = heights[state[2][0][1]]
    if rotation_face == 'F':
        top_left_horizontal_offset = heights[state[5][0][0]]  
        top_left_vertical_offset = heights[state[1][1][1]]   
        top_right_horizontal_offset = heights[state[0][1][0]]
        top_right_vertical_offset = heights[state[1][0][1]]   
        bottom_left_horizontal_offset = heights[state[5][0][1]] 
        bottom_left_vertical_offset = heights[state[3][1][0]]   
        bottom_right_horizontal_offset = heights[state[0][1][1]] 
        bottom_right_vertical_offset = heights[state[3][0][0]]
    if rotation_face == 'R':
        top_left_horizontal_offset = heights[state[2][0][1]]
        top_left_vertical_offset = heights[state[0][1][1]] 
        top_right_horizontal_offset = heights[state[4][0][0]]
        top_right_vertical_offset = heights[state[0][0][1]] 
        bottom_left_horizontal_offset = heights[state[2][1][1]] 
        bottom_left_vertical_offset = heights[state[5][0][1]]
        bottom_right_horizontal_offset = heights[state[4][1][0]]
        bottom_right_vertical_offset = heights[state[5][1][1]]
    
    horizontal_offset = (- top_left_horizontal_offset * top_left_volume - bottom_left_horizontal_offset * bottom_left_volume + top_right_horizontal_offset * top_right_volume + bottom_right_horizontal_offset * bottom_right_volume) / total_volume
    vertical_offset = (- top_left_vertical_offset * top_left_volume - bottom_left_vertical_offset * bottom_left_volume + top_right_vertical_offset * top_right_volume + bottom_right_vertical_offset * bottom_right_volume) / total_volume

    return horizontal_offset, vertical_offset

def main():
    print(assign_heights([0.02, 0.03, 0.02, 0.03, 0.04, 0.04]))

if __name__ == "__main__":
    main()