def get_grip_position(state, heights, rotation):
    rotation_face = rotation[0]
    if rotation_face == 'U':
        top_left = heights[state[1][0][0]]
        top_right = heights[state[3][0][1]]
        bottom_left = heights[state[1][0][1]]
        bottom_right = heights[state[3][0][0]]
    elif rotation_face  == 'F':
        top_left = heights[state[1][0][1]]
        top_right = heights[state[3][0][0]]
        bottom_left = heights[state[1][1][1]]
        bottom_right = heights[state[3][1][0]]
    elif rotation_face == 'R':
        top_left = heights[state[2][0][1]]
        top_right = heights[state[4][0][0]]
        bottom_left = heights[state[2][1][1]]
        bottom_right = heights[state[4][1][0]]
    else :
        raise ValueError
    
    top = top_left + top_right 
    bottom = bottom_left + bottom_right
    if top > bottom : # grip bottom
        grip_center = ((bottom_right - bottom_left)/2, -0.0)
    else: # grip top
        grip_center = ((top_right - top_left)/2, 0.0)
    
    return grip_center


def assign_heights(lengths):
    # lengths : the length of Up, Left, Front, Right, Back, and Down
    heights = {}
    for i in range(6):
        for j in range(4):
            length = lengths[i]
            heights[4 * i + j + 1] = length
    return heights

def main():
    print(assign_heights([0.02, 0.03, 0.02, 0.03, 0.04, 0.04]))

if __name__ == "__main__":
    main()