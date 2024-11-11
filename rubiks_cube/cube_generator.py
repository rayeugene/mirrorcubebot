import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
import copy

class CubeGenerator():
    def generate_mirror_2_by_2(edge_length, cube_center, mass_density, file_out="models/mirror_cube_2_by_2.sdf"):
        """
        args:
            edge_length: edge length of the cube in meters
            cube_center: (3,) np.array specifies the center (in meters) of rotation of the cube. This also dictates the size of all the cubies. 
                The green face is considered the x plane, the red face is considered the y plane, and the white face is considered the z plane.
                The cube lives in Quadrant 1.
            mass_density: scalar that maps from volume to mass
        returns: 
            None

        Generates 
        """
        sdf = ET.Element('sdf', {'version':'1.7'})
        model = ET.SubElement(sdf, 'model', {'name':'mirror_cube_2x2'})
        center_link = ET.SubElement(model, 'link', {'name':'center'})
        for x in range(2):
            for y in range(2):
                for z in range(2):
                    link_name = f'cubie_{x}_{y}_{z}'
                    parent_to_cubie = np.concatenate((-(cube_center - edge_length * np.array([x,y,z]))/2, np.zeros(3)))
                    cubie_size = 2*np.abs(parent_to_cubie[:3])
                    mass = np.linalg.norm(cubie_size)*mass_density
                    inertia_xx = mass/12 * (cubie_size[1]**2 + cubie_size[2]**2)
                    inertia_yy = mass/12 * (cubie_size[0]**2 + cubie_size[2]**2)
                    inertia_zz = mass/12 * (cubie_size[0]**2 + cubie_size[1]**2)
                    inertia = np.zeros(6)
                    inertia[0] = inertia_xx
                    inertia[3] = inertia_yy
                    inertia[5] = inertia_zz
                    rgba_color = np.array([192,192,192,255])/255.0
                    rgba_color_name = "silver"
                    model.append(CubeGenerator.generate_cubie(link_name, parent_to_cubie, cubie_size, mass, inertia, rgba_color, rgba_color_name))

                    joint_name = f'ball_{x}_{y}_{z}'
                    model.append(CubeGenerator.generate_joint(joint_name, 'ball', -parent_to_cubie, 'center', link_name))

        tree = ET.ElementTree(sdf)
        # tree.write(file_out)
        xml_str = ET.tostring(tree.getroot(), encoding='unicode')
        pretty_xml = minidom.parseString(xml_str).toprettyxml()
        with open(file_out, "w") as f:
            f.write(pretty_xml)

    def generate_cubie(link_name, pos_from_parent, cubie_size, mass, inertia, rgba_color, rgba_color_name=None):
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
        geometry = ET.Element('geometry')
        geometry_box = ET.SubElement(geometry, 'box')
        ET.SubElement(geometry_box, 'size').text = CubeGenerator.list_to_spaced_str(cubie_size)

        cubie_link = ET.Element('link', name=link_name)

        pose = ET.SubElement(cubie_link, 'pose')
        pose.text = CubeGenerator.list_to_spaced_str(pos_from_parent)

        inertial = ET.SubElement(cubie_link, 'inertial')
        ET.SubElement(inertial, 'mass').text = str(mass)
        inertial_inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertial_inertia, 'ixx').text = str(inertia[0])
        ET.SubElement(inertial_inertia, 'ixy').text = str(inertia[1])
        ET.SubElement(inertial_inertia, 'ixz').text = str(inertia[2])
        ET.SubElement(inertial_inertia, 'iyy').text = str(inertia[3])
        ET.SubElement(inertial_inertia, 'iyz').text = str(inertia[4])
        ET.SubElement(inertial_inertia, 'izz').text = str(inertia[5])

        collision = ET.SubElement(cubie_link, 'collision', {'name':'collision'})
        collision.append(copy.deepcopy(geometry))

        visual = ET.SubElement(cubie_link, 'visual')
        if rgba_color_name is not None:
            visual.set('name', rgba_color_name)
        visual.append(copy.deepcopy(geometry))
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'diffuse').text = CubeGenerator.list_to_spaced_str(rgba_color)

        return cubie_link

    def generate_joint(joint_name, joint_type, pose_to_parent, parent, child, damping=0.1, effort=0):
        joint = ET.Element('joint', {'name':joint_name, 'type':joint_type})

        ET.SubElement(joint, 'pose').text = CubeGenerator.list_to_spaced_str(pose_to_parent)
        ET.SubElement(joint, 'parent').text = parent
        ET.SubElement(joint, 'child').text = child

        axis = ET.SubElement(joint, 'axis')
        axis_dynamics = ET.SubElement(axis, 'dynamics')
        ET.SubElement(axis_dynamics, 'damping').text = str(damping)
        axis_limit = ET.SubElement(axis, 'limit')
        ET.SubElement(axis_limit, 'effort').text = str(effort)

        return joint

    def list_to_spaced_str(iterable):
        return " ".join(f"{x}" for x in iterable)

CubeGenerator.generate_mirror_2_by_2(0.10, np.ones(3)*0.05, 1920)

# sdf = ET.Element('sdf', {'version':'1.7'})
# ET.ElementTree(sdf).write("test.sdf")