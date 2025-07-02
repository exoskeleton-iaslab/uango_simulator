#!/usr/bin/env python3

import os
import xml.etree.ElementTree as Et

import numpy as np


class Links:
    def __init__(self):
        self.links_values = {}

    @staticmethod
    def _parse_inertial(xml_element):
        mass = None
        com_offset = None

        inertial = xml_element.find('inertial')
        if inertial is not None:
            mass_elem = inertial.find('mass')
            if mass_elem is not None:
                mass = float(mass_elem.attrib.get('value', 0.0))

            origin = inertial.find('origin')
            if origin is not None:
                xyz_str = origin.attrib.get('xyz', '0 0 0')
                xyz = [float(x) for x in xyz_str.split()]
                com_offset = np.array(xyz)

        return mass, com_offset

    @staticmethod
    def _generate_urdf_from_xacro(xacro_path, output_urdf_path):
        cmd = f"xacro {xacro_path} > {output_urdf_path}"
        try:
            os.system(cmd)
        except Exception as e:
            print(f"Error executing xacro command: {e}")

    def parse_links(self):
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        xacro_path = os.path.join(base_dir, "urdf", "robot.xacro")
        urdf_path = os.path.join(base_dir, "urdf", "robot.urdf")
        self._generate_urdf_from_xacro(xacro_path, urdf_path)

        if not os.path.exists(xacro_path):
            raise FileNotFoundError(f"Xacro file not found: {xacro_path}")
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")

        tree = Et.parse(urdf_path)
        root = tree.getroot()

        for link_elem in root.findall('link'):
            name = link_elem.attrib.get("name")
            mass, com_offset = self._parse_inertial(link_elem)
            if name and mass is not None and com_offset is not None:
                short_name = name.split('_')[0]
                self.links_values[short_name] = {
                    "mass": mass,
                    "com_offset": com_offset
                }

        try:
            os.remove(urdf_path)
        except Exception as e:
            print(f"Warning: failed to remove temporary URDF file: {e}")

        return self.links_values


# if __name__ == "__main__":
#     link_parser = Links()
#     links = link_parser.parse_links()
#     print(links)
