import copy
import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np

class BlockGenerator:

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
        ET.SubElement(geometry_box, "size").text = BlockGenerator.list_to_spaced_str(
            cubie_size
        )

        cubie_link = ET.Element("link", name=link_name)

        pose = ET.SubElement(cubie_link, "pose")
        pose.text = BlockGenerator.list_to_spaced_str(pos_from_parent)

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
        ET.SubElement(material, "diffuse").text = BlockGenerator.list_to_spaced_str(
            rgba_color
        )

        return cubie_link
    
    def generate_static_corner_block(
        edge_length: float,
        corner_position: np.array,
        mass_density: int,
        solid_color: dict = None,
        file_out: str = "models/static_corner_block.sdf",
    ):
        """
        Generates a static corner block with no joints.

        Args:
            edge_length: Edge length of the corner block in meters.
            corner_position: (3,) np.array specifies the position of the block.
            mass_density: Scalar that maps from volume to mass.
            solid_color: Optional, dict {'name': 'color_name', 'rgba': np.array([r, g, b, a])}.
            file_out: Output file path for the generated SDF file.
        """
        sdf = ET.Element("sdf", {"version": "1.7"})
        model = ET.SubElement(sdf, "model", {"name": "static_corner_block"})
        
        # Static property to prevent movement
        ET.SubElement(model, "static").text = "true"
        
        # Define the corner block as a single solid element
        block_size = np.ones(3) * edge_length
        mass = np.prod(block_size) * mass_density
        inertia_xx = mass / 12 * (block_size[1] ** 2 + block_size[2] ** 2)
        inertia_yy = mass / 12 * (block_size[0] ** 2 + block_size[2] ** 2)
        inertia_zz = mass / 12 * (block_size[0] ** 2 + block_size[1] ** 2)
        inertia = np.zeros(6)
        inertia[0] = inertia_xx
        inertia[3] = inertia_yy
        inertia[5] = inertia_zz
        
        rgba_color = (
            solid_color["rgba"] if solid_color else np.array([0.5, 0.5, 0.5, 1])
        )
        block_color_name = solid_color["name"] if solid_color else "gray"
        
        block = BlockGenerator.generate_cubie(
            "corner_block",
            corner_position,
            block_size,
            mass,
            inertia,
            rgba_color,
            block_color_name,
        )
        model.append(block)

        # Save the SDF
        tree = ET.ElementTree(sdf)
        xml_str = ET.tostring(tree.getroot(), encoding="unicode")
        pretty_xml = minidom.parseString(xml_str).toprettyxml()
        with open(file_out, "w") as f:
            f.write(pretty_xml)

    def list_to_spaced_str(iterable):
        return " ".join(f"{x}" for x in iterable)

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
