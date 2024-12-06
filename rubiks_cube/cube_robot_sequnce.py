from rubik_solver.scripts import cube_2x2x2, rubik_solver

class RobotCubeSequenceInterpreter:
    moves = ["F", "F'", "F2", "B", "B'", "B2", "R", "R'", "R2", "L", "L'", "L2", "U", "U'", "U2", "D", "D'", "D2"]
    robot_forbidden_moves = ["B", "B'", "B2", "L", "L'", "L2", "D", "D'", "D2"]
    forbidden_to_allowed_moves = {
        "B": "F",
        "B'": "F'",
        "B2": "F2",
        "L": "R",
        "L'": "R'",
        "L2": "R2",
        "D": "U",
        "D'": "U'",
        "D2": "U2",
    }
    
    def convert_sequence_to_robot_space(seq):
        # we are not allowing B, L, or D moves for now because the rear back cubie is anchored
        res = []
        for move in seq:
            if move in RobotCubeSequenceInterpreter.robot_forbidden_moves:
                res.append(RobotCubeSequenceInterpreter.forbidded_to_allowed_moves[move])
            else:
                res.append(move)
        return res
    