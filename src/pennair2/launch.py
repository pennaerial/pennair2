# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom
import rospkg
import os
import roslaunch


def launch(package, name, **kwargs):
    """Call roslaunch.

    :param package: The package name.
    :param name: The name of the launch file
    :param kwargs:
    """
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_args = [package, name]
    for key, value in kwargs.iteritems():
        cli_args.append(key + ":=" + value)  # 'arg1:=arg1', 'arg2:=arg2' ...

    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    roslaunch_args = cli_args[2:]
    parent = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file, roslaunch_args)])
    parent.start()

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
        path += "/" + name
        with open(path, "w") as f:
            f.write(self.generate())
