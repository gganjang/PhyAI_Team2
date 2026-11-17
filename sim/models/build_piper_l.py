#!/usr/bin/env python3
"""Build the lab's Menagerie-compatible PiPER-L MJCF from an official URDF checkout.

Requires mujoco and pycollada (see requirements.txt). Generated assets are external
build products; this does not modify the upstream Menagerie or URDF repository.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import collada
import mujoco
import numpy as np


PACKAGE_PREFIX = 'package://agx_arm_description/agx_arm_urdf/'
RENAMES = {'flange_link': 'link6_flange', 'gripper_base': 'link6_gripper_base',
           'gripper_link1': 'link7', 'gripper_link2': 'link8',
           'gripper_joint1': 'joint7', 'gripper_joint2': 'joint8'}


def vector(values):
    return ' '.join(format(float(v), '.12g') for v in values)


def origin(element):
    """URDF uses fixed-axis roll/pitch/yaw; MJCF stores w,x,y,z quaternions."""
    if element is None:
        return {'pos': '0 0 0', 'quat': '1 0 0 0'}
    roll, pitch, yaw = [float(v) / 2 for v in element.get('rpy', '0 0 0').split()]
    cr, cp, cy = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sr, sp, sy = math.sin(roll), math.sin(pitch), math.sin(yaw)
    return {'pos': element.get('xyz', '0 0 0'),
            'quat': vector([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                            cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])}


def mesh_path(source, filename):
    if not filename.startswith(PACKAGE_PREFIX):
        raise ValueError(f'Unexpected mesh URI: {filename}')
    path = (source / filename.removeprefix(PACKAGE_PREFIX)).resolve()
    if not path.is_relative_to(source.resolve()):
        raise ValueError(f'Mesh escapes source directory: {filename}')
    return path


def visual_meshes(path, assets, asset_node, prefix):
    """Bake COLLADA scene transforms and split by material for MuJoCo's OBJ loader."""
    document = collada.Collada(str(path))
    if document.assetInfo.unitmeter != 1 or document.assetInfo.upaxis != 'Z_UP':
        raise ValueError(f'Expected metre/Z-up COLLADA: {path}')
    result = []
    for geometry in document.scene.objects('geometry'):
        for primitive in geometry.primitives():
            triangles = primitive.triangleset() if hasattr(primitive, 'triangleset') else primitive
            name = f'{prefix}_visual_{len(result)}'
            with (assets / f'{name}.obj').open('w') as output:
                for vertex in triangles.vertex:
                    output.write('v ' + vector(vertex) + '\n')
                for face in triangles.vertex_index:
                    output.write('f ' + ' '.join(str(int(v) + 1) for v in face) + '\n')
            diffuse = triangles.material.effect.diffuse
            if not isinstance(diffuse, tuple):
                raise ValueError(f'Unsupported COLLADA diffuse material: {path}')
            # Decorative labels may be planar; their geoms never contribute inertia.
            ET.SubElement(asset_node, 'mesh', name=name, file=f'{name}.obj', inertia='shell')
            result.append((name, vector(diffuse)))
    if not result:
        raise ValueError(f'No visual triangles: {path}')
    return result


def build(source, output, revision):
    source, output = Path(source), Path(output)
    # The gripper xacro contains only an include and literal link/joint elements;
    # load the included base explicitly rather than requiring a ROS/xacro runtime.
    arm = ET.parse(source / 'piper_l/urdf/piper_l_description.urdf').getroot()
    gripper = ET.parse(source / 'piper_l/urdf/piper_l_with_gripper_description.xacro').getroot()
    links = {link.get('name'): link for doc in (arm, gripper) for link in doc.findall('link')}
    joints = [j for doc in (arm, gripper) for j in doc.findall('joint')
              if j.get('name') not in ('world_to_base_link', 'gripper')]
    children = {}
    for joint in joints:
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    output.mkdir(parents=True, exist_ok=True)
    assets = output / 'assets'
    assets.mkdir(exist_ok=True)
    root = ET.Element('mujoco', model='piper_l_description')
    ET.SubElement(root, 'compiler', angle='radian', meshdir='assets', autolimits='true')
    ET.SubElement(root, 'option', integrator='implicitfast', cone='elliptic', impratio='10')
    defaults = ET.SubElement(root, 'default')
    piper = ET.SubElement(defaults, 'default', {'class': 'piper'})
    ET.SubElement(piper, 'joint', frictionloss='0.3', armature='0.005')
    ET.SubElement(piper, 'position', inheritrange='1', forcerange='-100 100')
    finger = ET.SubElement(piper, 'default', {'class': 'finger'})
    ET.SubElement(finger, 'joint', frictionloss='0', type='slide')
    ET.SubElement(finger, 'position', forcerange='-10 10')
    visual = ET.SubElement(piper, 'default', {'class': 'visual'})
    ET.SubElement(visual, 'geom', group='2', type='mesh', contype='0', conaffinity='0', density='0')
    collision = ET.SubElement(piper, 'default', {'class': 'collision'})
    ET.SubElement(collision, 'geom', group='3', type='mesh')
    asset_node = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')

    def add_link(parent, name, joint=None):
        body_name = RENAMES.get(name, name)
        attrs = origin(joint.find('origin') if joint is not None else None)
        body = ET.SubElement(parent, 'body', name=body_name, gravcomp='1', **attrs)
        if joint is None:
            body.set('childclass', 'piper')
        if body_name == 'link7':
            # Keep the lab mount convention: +X above the wrist, +Y toward
            # the fingertips. Official finger frames differ from Menagerie
            # by a half-turn around Y. This site defines a mount frame only;
            # joint, mesh and inertial frames remain the official URDF frames.
            ET.SubElement(body, 'site', name='wrist_mount_frame', quat='0 0 1 0',
                          size='.001', rgba='0 0 0 0', group='5')
        link = links[name]
        inertial = link.find('inertial')
        if inertial is not None:
            pose = origin(inertial.find('origin'))
            tensor = inertial.find('inertia')
            matrix = np.array([[float(tensor.get('ixx')), float(tensor.get('ixy')), float(tensor.get('ixz'))],
                               [float(tensor.get('ixy')), float(tensor.get('iyy')), float(tensor.get('iyz'))],
                               [float(tensor.get('ixz')), float(tensor.get('iyz')), float(tensor.get('izz'))]])
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, np.fromstring(pose.pop('quat'), sep=' '))
            rotation = rotation.reshape(3, 3)
            matrix = rotation @ matrix @ rotation.T
            ET.SubElement(body, 'inertial', mass=inertial.find('mass').get('value'), **pose,
                          fullinertia=vector(matrix[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]))
        if joint is not None and joint.get('type') != 'fixed':
            limit = joint.find('limit')
            joint_name = RENAMES.get(joint.get('name'), joint.get('name'))
            ET.SubElement(body, 'joint', name=joint_name, axis=joint.find('axis').get('xyz'),
                          range=f"{limit.get('lower')} {limit.get('upper')}",
                          **({'class': 'finger'} if joint.get('type') == 'prismatic' else {}))
        for index, visual in enumerate(link.findall('visual')):
            path = mesh_path(source, visual.find('geometry/mesh').get('filename'))
            for mesh, rgba in visual_meshes(path, assets, asset_node, f'{body_name}_{index}'):
                ET.SubElement(body, 'geom', {'class': 'visual', 'mesh': mesh, 'rgba': rgba,
                                           **origin(visual.find('origin'))})
        for index, collision in enumerate(link.findall('collision')):
            path = mesh_path(source, collision.find('geometry/mesh').get('filename'))
            mesh = f'{body_name}_collision_{index}'
            shutil.copyfile(path, assets / f'{mesh}.stl')
            ET.SubElement(asset_node, 'mesh', name=mesh, file=f'{mesh}.stl')
            ET.SubElement(body, 'geom', {'class': 'collision', 'mesh': mesh,
                                       **origin(collision.find('origin'))})
        for child_joint in children.get(name, []):
            add_link(body, child_joint.find('child').get('link'), child_joint)

    add_link(world, 'base_link')
    contact = ET.SubElement(root, 'contact')
    ET.SubElement(contact, 'exclude', body1='base_link', body2='link1')
    # The fixed flange and gripper base are one rigid assembly with link6.
    # MuJoCo already filters welded parent bodies; these exclusions also keep
    # their intentional overlap with the wrist housing out of self contacts.
    for body in ('link6_flange', 'link6_gripper_base'):
        ET.SubElement(contact, 'exclude', body1='link5', body2=body)
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'joint', joint1='joint8', joint2='joint7', polycoef='0 -1 0 0 0')
    actuators = ET.SubElement(root, 'actuator')
    for i, (kp, kv) in enumerate(((80, 5), (80, 5), (80, 5), (40, 5), (10, 1.5), (10, 1.5)), 1):
        ET.SubElement(actuators, 'position', {'name': f'joint{i}', 'joint': f'joint{i}',
                                           'class': 'piper', 'kp': str(kp), 'kv': str(kv)})
    ET.SubElement(actuators, 'position', {'name': 'gripper', 'joint': 'joint7',
                                       'class': 'finger', 'kp': '40', 'kv': '5'})
    keys = ET.SubElement(root, 'keyframe')
    ET.SubElement(keys, 'key', name='home', qpos='0 1.57 -1.3485 0 0 0 0 0', ctrl='0 1.57 -1.3485 0 0 0 0')
    ET.indent(root)
    xml_path = output / 'piper.xml'
    ET.ElementTree(root).write(xml_path, encoding='unicode')
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    if model.nq != 8 or model.nu != 7:
        raise ValueError('PiPER-L must expose eight joint positions and seven controls')
    shutil.copyfile(source / 'LICENSE', output / 'LICENSE.agx_arm_urdf')
    (output / 'provenance.json').write_text(json.dumps({
        'model': 'PiPER-L', 'source': 'https://github.com/agilexrobotics/agx_arm_urdf',
        'revision': revision, 'arm': 'piper_l/urdf/piper_l_description.urdf',
        'gripper': 'piper_l/urdf/piper_l_with_gripper_description.xacro',
        'collision': 'Official STL meshes, convex hulls in MuJoCo',
        'control': 'Menagerie position gains, armature, friction and gravity compensation; not hardware identification',
    }, indent=2) + '\n')
    return xml_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--revision', required=True)
    args = parser.parse_args()
    print(build(args.source, args.output, args.revision))


if __name__ == '__main__':
    main()
