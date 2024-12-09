import numpy as np

class CubeScrambler:
    moves = ["F", "F'", "F2", "B", "B'", "B2", "R", "R'", "R2", "L", "L'", "L2", "U", "U'", "U2", "D", "D'", "D2"]
    forbidden_to_allowed_moves = {
        "B": "F",
        "B'": "F'",
        "B2": ["F", "F"],
        "L": "R",
        "L'": "R'",
        "L2": ["R", "R"],
        "D": "U",
        "D'": "U'",
        "D2": ["U", "U"],
        "R2": ["R", "R"],
        "U2": ["U", "U"],
        "F2": ["F", "F"],
    }

    def __init__(self):        
        self.ball_joint_poses = np.zeros((2,2,2,3))
        self.cubie_idx_to_ball_joint_idx = np.empty((2,2,2), dtype=object)
        for x in range(2):
            for y in range(2):
                for z in range(2):
                    self.cubie_idx_to_ball_joint_idx[x,y,z] = (int(x),int(y),int(z))
        # print(self.cubie_idx_to_ball_joint_idx)
    
    def convert_sequence_to_robot_space(self, seq):
        # we are not allowing B, L, or D moves for now because the rear back cubie is anchored
        res = []
        for move in seq:
            if move in CubeScrambler.forbidden_to_allowed_moves:
                if isinstance(CubeScrambler.forbidden_to_allowed_moves[move], list):
                    res.extend(CubeScrambler.forbidden_to_allowed_moves[move])
                else:
                    res.append(CubeScrambler.forbidden_to_allowed_moves[move])
            else:
                res.append(move)
        return res
    
    def apply_sequence(self, seq):
        for move in seq:
            self.apply_move(move)

    def apply_move(self, move:str):
        if "F" in move:
            F_cubie_idxs = self.cubie_idx_to_ball_joint_idx[0,:,:].flatten()
            # print('f_cubie_idxs',F_cubie_idxs)
            tmp = self.cubie_idx_to_ball_joint_idx[0,0,0]
            if "'" in move:
                for idx in F_cubie_idxs:
                    self.ball_joint_poses[idx][0] = (self.ball_joint_poses[idx][1] - np.pi/2) % (2*np.pi)
                self.cubie_idx_to_ball_joint_idx[0,0,0] = self.cubie_idx_to_ball_joint_idx[0,1,0]
                self.cubie_idx_to_ball_joint_idx[0,1,0] = self.cubie_idx_to_ball_joint_idx[0,1,1]
                self.cubie_idx_to_ball_joint_idx[0,1,1] = self.cubie_idx_to_ball_joint_idx[0,0,1]
                self.cubie_idx_to_ball_joint_idx[0,0,1] = tmp
            else:
                for idx in F_cubie_idxs:
                    self.ball_joint_poses[idx][0] = (self.ball_joint_poses[idx][1] + np.pi/2) % (2*np.pi)
                self.cubie_idx_to_ball_joint_idx[0,0,0] = self.cubie_idx_to_ball_joint_idx[0,0,1]
                self.cubie_idx_to_ball_joint_idx[0,0,1] = self.cubie_idx_to_ball_joint_idx[0,1,1]
                self.cubie_idx_to_ball_joint_idx[0,1,1] = self.cubie_idx_to_ball_joint_idx[0,1,0]
                self.cubie_idx_to_ball_joint_idx[0,1,0] = tmp
        elif "R" in move:
            R_cubie_idxs = self.cubie_idx_to_ball_joint_idx[:,0,:].flatten()
            tmp = self.cubie_idx_to_ball_joint_idx[0,0,0]
            if "'" in move:
                for idx in R_cubie_idxs:
                    self.ball_joint_poses[idx][1] = (self.ball_joint_poses[idx][0] + np.pi/2) % (2*np.pi)
                self.cubie_idx_to_ball_joint_idx[0,0,0] = self.cubie_idx_to_ball_joint_idx[0,0,1]
                self.cubie_idx_to_ball_joint_idx[0,0,1] = self.cubie_idx_to_ball_joint_idx[0,1,1]
                self.cubie_idx_to_ball_joint_idx[0,1,1] = self.cubie_idx_to_ball_joint_idx[0,1,0]
                self.cubie_idx_to_ball_joint_idx[0,1,0] = tmp
            else:
                for idx in R_cubie_idxs:
                    self.ball_joint_poses[idx][1] = (self.ball_joint_poses[idx][0] - np.pi/2) % (2*np.pi)
                self.cubie_idx_to_ball_joint_idx[0,0,0] = self.cubie_idx_to_ball_joint_idx[0,1,0]
                self.cubie_idx_to_ball_joint_idx[0,1,0] = self.cubie_idx_to_ball_joint_idx[0,1,1]
                self.cubie_idx_to_ball_joint_idx[0,1,1] = self.cubie_idx_to_ball_joint_idx[0,0,1]
                self.cubie_idx_to_ball_joint_idx[0,0,1] = tmp
        else:
            U_cubie_idxs = self.cubie_idx_to_ball_joint_idx[:,:,1].flatten()
            tmp = self.cubie_idx_to_ball_joint_idx[0,0,1]
            if "'" in move:
                for idx in U_cubie_idxs:
                    self.ball_joint_poses[idx][2] = (self.ball_joint_poses[idx][2] + np.pi/2) % (2*np.pi)
                self.cubie_idx_to_ball_joint_idx[0,0,1] = self.cubie_idx_to_ball_joint_idx[1,0,1]
                self.cubie_idx_to_ball_joint_idx[1,0,1] = self.cubie_idx_to_ball_joint_idx[1,1,1]
                self.cubie_idx_to_ball_joint_idx[1,1,1] = self.cubie_idx_to_ball_joint_idx[0,1,1]
                self.cubie_idx_to_ball_joint_idx[0,1,1] = tmp
            else:
                for idx in U_cubie_idxs:
                    self.ball_joint_poses[idx][2] = (self.ball_joint_poses[idx][2] - np.pi/2) % (2*np.pi)
                self.cubie_idx_to_ball_joint_idx[0,0,1] = self.cubie_idx_to_ball_joint_idx[0,1,1]
                self.cubie_idx_to_ball_joint_idx[0,1,1] = self.cubie_idx_to_ball_joint_idx[1,1,1]
                self.cubie_idx_to_ball_joint_idx[1,1,1] = self.cubie_idx_to_ball_joint_idx[1,0,1]
                self.cubie_idx_to_ball_joint_idx[1,0,1] = tmp
            

    def get_joint_rpys(self):
        return self.ball_joint_poses # roll pitch yaw
    
def main():
    cube_scrambler = CubeScrambler()
    cube_scrambler.apply_move('R')
    # print(cube_scrambler.get_joint_rpys())

if __name__ == "__main__":
    main()