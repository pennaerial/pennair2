from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom
import rospkg
import os

class LaunchFile:
    def __init__(self):
        self.element = Element("launch")
        self.nodes = {}

    def add_node(self, name, node):
        # type: (str, xml.Element) -> None
        self.element.append(node.element)
        self.nodes[name] = node

    class Node:
        def __init__(self, name, params):
            self.name = name  # type: str
            self.params = params  # type: dict
            self.element = Element("node")  # type: xml.Element

        def add_param(self, name, value):
            SubElement(self.element, "param", {"name": name, "value": value})

        def add_remap(self, old, new):
            SubElement(self.element, "remap", {"from": old, "to": new})

        def add_rosparam(self, name, text):
            rosparam = SubElement(self.element, "rosparam", {"param": name})
            rosparam.text = text

    def generate(self):
        reparsed = minidom.parseString(tostring(self.element))
        return reparsed.toprettyxml(indent="    ")

    def write(self, package, name):
        rospack = rospkg.RosPack()
        path = rospack.get_path(package) + "/launch"
        if not os.path.exists(path):
            os.makedirs(path)
        path += "/" + name + ".launch"
        with open(path, "w") as f:
            f.write(self.generate())