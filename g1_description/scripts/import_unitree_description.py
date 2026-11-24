#!/usr/bin/env python3
"""Import G1 URDFs and their meshes from Unitree's unitree_ros into this package.

Mesh paths are rewritten from "meshes/x.STL" to "package://g1_description/meshes/x.STL",
so the URDF that robot_state_publisher publishes on the robot also resolves in RViz on
another computer that has this package installed. Only the meshes the URDFs use are copied.

  ./import_unitree_description.py ~/git/unitree_ros/robots/g1_description \
      g1_29dof_lock_waist_rev_1_0 g1_29dof_rev_1_0
"""

import argparse
import os
import shutil
import subprocess
import xml.etree.ElementTree as ElementTree

PACKAGE = 'g1_description'
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def strip_floating_base(robot):
    """Drop floating joints whose parent is an empty root link (e.g. 'world')."""
    links = {link.get('name'): link for link in robot.findall('link')}
    children = {joint.find('child').get('link') for joint in robot.findall('joint')}
    for joint in robot.findall('joint'):
        parent = joint.find('parent').get('link')
        if (joint.get('type') == 'floating' and parent not in children
                and parent in links and len(list(links[parent])) == 0):
            robot.remove(joint)
            robot.remove(links[parent])


def import_model(source_dir, model):
    tree = ElementTree.parse(os.path.join(source_dir, f'{model}.urdf'))
    robot = tree.getroot()
    strip_floating_base(robot)
    meshes = set()
    for mesh in robot.iter('mesh'):
        filename = mesh.get('filename', '')
        if '://' in filename:
            continue
        meshes.add(filename)
        mesh.set('filename', f'package://{PACKAGE}/{filename}')
    tree.write(os.path.join(PACKAGE_DIR, 'urdf', f'{model}.urdf'), encoding='unicode',
               xml_declaration=True)
    return meshes


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('source_dir', help='unitree_ros/robots/g1_description')
    parser.add_argument('models', nargs='+', help='URDF names without .urdf')
    args = parser.parse_args()
    source_dir = os.path.abspath(os.path.expanduser(args.source_dir))

    os.makedirs(os.path.join(PACKAGE_DIR, 'urdf'), exist_ok=True)
    os.makedirs(os.path.join(PACKAGE_DIR, 'meshes'), exist_ok=True)
    meshes = set()
    for model in args.models:
        meshes |= import_model(source_dir, model)
    for mesh in sorted(meshes):
        shutil.copy2(os.path.join(source_dir, mesh), os.path.join(PACKAGE_DIR, mesh))

    commit = subprocess.run(['git', '-C', source_dir, 'rev-parse', 'HEAD'],
                            capture_output=True, text=True).stdout.strip() or 'unknown'
    with open(os.path.join(PACKAGE_DIR, 'SOURCE'), 'w') as source:
        source.write(f'https://github.com/unitreerobotics/unitree_ros robots/g1_description\n'
                     f'commit {commit}\nmodels {" ".join(args.models)}\n')
    print(f'Imported {len(args.models)} URDF(s) and {len(meshes)} meshes (commit {commit[:7]})')


if __name__ == '__main__':
    main()
