import copy
import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np


class CubeGenerator:
    sticker_color_dict = {
        0: {
            1: {"name": "orange", "rgba": np.array([1, 0.5, 0, 1])},
            -1: {"name": "red", "rgba": np.array([1, 0, 0, 1])},
        },
        1: {
            1: {"name": "blue", "rgba": np.array([0, 0, 1, 1])},
            -1: {"name": "green", "rgba": np.array([0, 1, 0, 1])},
        },
        2: {
            1: {"name": "yellow", "rgba": np.array([1, 1, 0, 1])},
            -1: {"name": "white", "rgba": np.array([1, 1, 1, 1])},
        },
    }

    def generate_mirror_2_by_2(
        edge_length: int,
        cube_center: np.array,
        mass_density: int,
        solid_color: dict = None,
        file_out: str = "models/mirror_cube_2_by_2.sdf",
    ):
        """
        args:
            edge_length: edge length of the cube in meters
            cube_center: (3,) np.array specifies the center (in meters) of rotation of the cube. 
                This also dictates the size of all the cubies.
                The green face is considered the x plane, the red face is considered the y plane, 
                and the white face is considered the z plane.
                The cube lives in Quadrant 1.
            mass_density: scalar that maps from volume to mass
            solid_color: optional, dict {'name':'color_name','rgba':np.array([r,g,b,a])}
        returns:
            None

        Generates
        """
        sdf = ET.Element("sdf", {"version": "1.7"})
        model = ET.SubElement(sdf, "model", {"name": "mirror_cube_2x2"})
        ET.SubElement(model, "link", {"name": "center"})
        for x in range(2):
            for y in range(2):
                for z in range(2):
                    link_name = f"cubie_{x}_{y}_{z}"
                    parent_to_cubie = np.concatenate(
                        (
                            -(cube_center - edge_length * np.array([x, y, z])) / 2,
                            np.zeros(3),
                        )
                    )
                    cubie_size = 2 * np.abs(parent_to_cubie[:3])
                    mass = np.prod(cubie_size[:3]) * mass_density
                    inertia_xx = mass / 12 * (cubie_size[1] ** 2 + cubie_size[2] ** 2)
                    inertia_yy = mass / 12 * (cubie_size[0] ** 2 + cubie_size[2] ** 2)
                    inertia_zz = mass / 12 * (cubie_size[0] ** 2 + cubie_size[1] ** 2)
                    inertia = np.zeros(6)
                    inertia[0] = inertia_xx
                    inertia[3] = inertia_yy
                    inertia[5] = inertia_zz
                    rgba_color = np.array([0, 0, 0, 1])
                    rgba_core_color_name = "black"
                    cubie = CubeGenerator.generate_cubie(
                        link_name,
                        parent_to_cubie,
                        cubie_size,
                        mass,
                        inertia,
                        rgba_color,
                        rgba_core_color_name,
                    )

                    # this will have to change for a nxn cube
                    cubie_stickers = CubeGenerator.generate_cubie_stickers(
                        np.array(
                            [
                                -1 if x == 0 else 1,
                                -1 if y == 0 else 1,
                                -1 if z == 0 else 1,
                            ]
                        ),
                        cubie_size,
                        solid_color,
                    )
                    for sticker in cubie_stickers:
                        cubie.append(sticker)
                    model.append(cubie)

                    joint_name = f"ball_{x}_{y}_{z}"
                    model.append(
                        CubeGenerator.generate_joint(
                            joint_name, "ball", -parent_to_cubie, "center", link_name
                        )
                    )

        tree = ET.ElementTree(sdf)
        xml_str = ET.tostring(tree.getroot(), encoding="unicode")
        pretty_xml = minidom.parseString(xml_str).toprettyxml()
        with open(file_out, "w") as f:
            f.write(pretty_xml)

    def generate_cubie(
        link_name: str,
        pos_from_parent: np.array,
        cubie_size: np.array,
        mass: int,
        inertia: np.array,
        rgba_color: np.array,
        rgba_core_color_name: str = None,
    ):
        """
        args:
            link_name: str
            pos_from_parent: 6x, in meters from parent element
            cubie_size: 3x, array meters
            mass: scalar in kg
            inertia: 6x, xx, xy, xz, yy, yz, zz
            rgba_color: 4x, float 0-1 for rgba diffuse tag
        returns:
            ET_Element
        """
        geometry = ET.Element("geometry")
        geometry_box = ET.SubElement(geometry, "box")
        ET.SubElement(geometry_box, "size").text = CubeGenerator.list_to_spaced_str(
            cubie_size
        )

        cubie_link = ET.Element("link", name=link_name)

        pose = ET.SubElement(cubie_link, "pose")
        pose.text = CubeGenerator.list_to_spaced_str(pos_from_parent)

        inertial = ET.SubElement(cubie_link, "inertial")
        ET.SubElement(inertial, "mass").text = str(mass)
        inertial_inertia = ET.SubElement(inertial, "inertia")
        ET.SubElement(inertial_inertia, "ixx").text = str(inertia[0])
        ET.SubElement(inertial_inertia, "ixy").text = str(inertia[1])
        ET.SubElement(inertial_inertia, "ixz").text = str(inertia[2])
        ET.SubElement(inertial_inertia, "iyy").text = str(inertia[3])
        ET.SubElement(inertial_inertia, "iyz").text = str(inertia[4])
        ET.SubElement(inertial_inertia, "izz").text = str(inertia[5])

        collision = ET.SubElement(cubie_link, "collision", {"name": "collision"})
        collision.append(copy.deepcopy(geometry))

        visual = ET.SubElement(cubie_link, "visual")
        if rgba_core_color_name is not None:
            visual.set("name", rgba_core_color_name)
        visual.append(copy.deepcopy(geometry))
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "diffuse").text = CubeGenerator.list_to_spaced_str(
            rgba_color
        )

        return cubie_link

    def generate_joint(
        joint_name, joint_type, pose_to_parent, parent, child, damping=0.1, effort=0
    ):
        joint = ET.Element("joint", {"name": joint_name, "type": joint_type})

        ET.SubElement(joint, "pose").text = CubeGenerator.list_to_spaced_str(
            pose_to_parent
        )
        ET.SubElement(joint, "parent").text = parent
        ET.SubElement(joint, "child").text = child

        axis = ET.SubElement(joint, "axis")
        axis_dynamics = ET.SubElement(axis, "dynamics")
        ET.SubElement(axis_dynamics, "damping").text = str(damping)
        axis_limit = ET.SubElement(axis, "limit")
        ET.SubElement(axis_limit, "effort").text = str(effort)

        return joint

    def generate_cubie_stickers(
        color_vector: np.array, cubie_dims: np.array, solid_sticker_color=None
    ):
        """
        args:
            color_vector: 3x, numpy array with 1,0,-1 encoding to indicate color 
            corresponding to the axis normal to a given face. For example,
                (1,1,-1) indicates orange on the face with largest x coordinate, 
                blue on the face with the largest y coordinate, and white on the 
                face with the smallest z coordinate.
                0 indicates no color for the given face perpendicular to that axis. 
                (1,0,1) would be an orange,yellow edge piece for example.
            cubie_dims: 3x, numpy array for dims of cubie in meters
            solid_sticker_color: dict {'name': 'color_name', 'rgba': 'rgba_vector'}
        returns:
            list: of ET element consisting of all three stickers
        """
        cubie_faces = []
        for x in range(3):
            if color_vector[x] == 0:
                continue
            sticker = ET.Element(
                "visual",
                name=(
                    CubeGenerator.sticker_color_dict[x][color_vector[x]]["name"]
                    if solid_sticker_color is None
                    else f'{solid_sticker_color["name"]}_{x}'
                ),
            )
            sticker_pose = np.zeros(6)
            sticker_pose[x] = color_vector[x] * cubie_dims[x] / 2
            ET.SubElement(sticker, "pose").text = CubeGenerator.list_to_spaced_str(
                sticker_pose
            )
            sticker_geometry = ET.SubElement(sticker, "geometry")
            sticker_geometry_box = ET.SubElement(sticker_geometry, "box")
            sticker_size_mult_mask = np.ones(3)
            sticker_size_mult_mask[x] = 0.01
            sticker_size = np.round(
                (cubie_dims - 0.005 * np.ones(3)) * sticker_size_mult_mask, 10
            )
            ET.SubElement(sticker_geometry_box, "size").text = (
                CubeGenerator.list_to_spaced_str(sticker_size)
            )
            sticker_material = ET.SubElement(sticker, "material")
            ET.SubElement(sticker_material, "diffuse").text = (
                CubeGenerator.list_to_spaced_str(
                    CubeGenerator.sticker_color_dict[x][color_vector[x]]["rgba"]
                    if solid_sticker_color is None
                    else solid_sticker_color["rgba"]
                )
            )
            cubie_faces.append(sticker)
        return cubie_faces

    def list_to_spaced_str(iterable):
        return " ".join(f"{x}" for x in iterable)


# CubeGenerator.generate_mirror_2_by_2(0.10, np.ones(3) * 0.05, 390) # normal cube with edge length 0.10
# CubeGenerator.generate_mirror_2_by_2(0.06, np.array([0.02,0.03, 0.04]), 390, solid_color={'name':'silver','rgba':np.array([193/255.0,193/255.0,193/255.0,1])}) # silver mirror cube, 0.06 edge length
# CubeGenerator.generate_mirror_2_by_2(
#     0.06, np.array([0.02, 0.03, 0.04]), 390
# )  # colored mirror cube, 0.06 edge length
CubeGenerator.generate_mirror_2_by_2(
    0.06, np.array([0.02, 0.03, 0.04]), 0.39, file_out = "models/light_mirror_cube_2_by_2.sdf"
)  # colored mirror cube, 0.06 edge length

# sdf = ET.Element('sdf', {'version':'1.7'})
# ET.ElementTree(sdf).write("test.sdf")
