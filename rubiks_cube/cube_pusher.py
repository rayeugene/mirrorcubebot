import numpy as np
from pydrake.all import (LeafSystem, Value, List, ExternallyAppliedSpatialForce, SpatialForce)

class CubePusher(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        forces_cls = Value[List[ExternallyAppliedSpatialForce]]
        self.DeclareAbstractOutputPort(
            "applied_force",
            lambda: forces_cls(),
            self.CalcOutput)

    def CalcOutput(self, context, output):
        forces = []
        for x,y,z in product([0,1], repeat=3):
            force = ExternallyAppliedSpatialForce()
            force.body_index = plant.GetBodyByName(f"box_{x}_{y}_{z}").index()
            # shift from [0, 1] to [-1, 1]
            x = 2 * x - 1
            y = 2 * y - 1
            z = 2 * z - 1
            force.p_BoBq_B = -0.0125*np.array([x, y, z]) # world 0, 0, 0
            if rotate_about == 'x':
                force.F_Bq_W = SpatialForce(
                    tau=-0.2 * np.array([1 if x < 0 else -1, 0, 0]),
                    f=[0, 0, 0])
            elif rotate_about == 'y':
                force.F_Bq_W = SpatialForce(
                    tau=0.2 * np.array([0, 1 if y > 0 else -1, 0]),
                    f=[0, 0, 0])
            else:
                force.F_Bq_W = SpatialForce(
                    tau=0.2 * np.array([0, 0, 1 if z > 0 else -1]),
                    f=[0, 0, 0])
        forces.append(force)
        output.set_value(forces)