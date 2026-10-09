"""Export a small known model built from fake Fusion objects and check the URDF.

Model: base_link at the origin, link1 on a revolute joint 0.1 m above it about z
(limits +-pi/2), and two occurrences of a 'wheel' component on fixed joints. Fusion
API units are cm, kg, and kg cm^2.

Run with: python -m pytest tests
"""
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from Fusion_URDF_Exporter_ROS2.core import Joint, Link, Write
from Fusion_URDF_Exporter_ROS2.utils import utils

IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


class FakeList(list):
    @property
    def count(self) -> int:
        return len(self)


def vec(xyz):
    return NS(asArray=lambda: list(xyz))


class Design:
    def __init__(self, params: dict[str, float]):
        self.rootComponent = None
        self.userParameters = NS(itemByName=lambda n: NS(value=params[n]) if n in params else None)


def make_model(params: dict[str, float] | None = None):
    design = Design(params or {})
    comps = {n: NS(name=n, parentDesign=design) for n in ('base_link', 'link1', 'wheel')}

    def occ(name, comp, mass, com_cm, moments):
        props = NS(mass=mass, centerOfMass=vec(com_cm),
                   getXYZMomentsOfInertia=lambda: (True, *moments))
        return NS(name=name, component=comps[comp],
                  transform=NS(translation=vec([0, 0, 0]), asArray=lambda: IDENTITY),
                  getPhysicalProperties=lambda accuracy: props)

    # moments are about the world origin, so give the small link its COM at the
    # origin to keep the expected values simple: 1e-4 kg cm^2 = 1e-8 kg m^2
    occs = FakeList([
        occ('base_link:1', 'base_link', 1.0, [0, 0, 0], [10, 10, 10, 0, 0, 0]),
        occ('link1:1', 'link1', 0.01, [0, 0, 0], [1e-4, 1e-4, 1e-4, 0, 0, 0]),
        occ('wheel:1', 'wheel', 0.1, [0, 0, 0], [1, 1, 1, 0, 0, 0]),
        occ('wheel:2', 'wheel', 0.1, [0, 0, 0], [1, 1, 1, 0, 0, 0]),
    ])
    by_name = {o.name: o for o in occs}

    def joint(name, kind, child, parent, origin_cm, axis=(0, 0, 1), limits=None):
        enabled = limits is not None
        lo, hi = limits or (0.0, 0.0)
        motion = NS(jointType=kind, rotationAxisVector=vec(axis),
                    rotationLimits=NS(isMaximumValueEnabled=enabled, isMinimumValueEnabled=enabled,
                                      maximumValue=hi, minimumValue=lo))
        return NS(name=name, jointMotion=motion,
                  occurrenceOne=by_name[child], occurrenceTwo=by_name[parent],
                  geometryOrOriginOne=NS(origin=vec(origin_cm)),
                  geometryOrOriginTwo=NS(origin=vec(origin_cm)))

    joints = FakeList([
        joint('joint1', 1, 'link1:1', 'base_link:1', [0, 0, 10], limits=(-math.pi / 2, math.pi / 2)),
        joint('wheel_left', 0, 'wheel:1', 'base_link:1', [0, 5, 0]),
        joint('wheel_right', 0, 'wheel:2', 'base_link:1', [0, -5, 0]),
    ])
    root = NS(occurrences=occs, joints=joints, parentDesign=design,
              allOccurrencesByComponent=lambda c: FakeList(o for o in occs if o.component is c))
    design.rootComponent = root
    return root


def export(tmp_path: Path, fixed_base: bool, params: dict[str, float] | None = None) -> ET.Element:
    root = make_model(params)
    msg = 'Successfully created URDF file'
    joints_dict, msg_j = Joint.make_joints_dict(root, msg)
    inertial_dict, msg_l = Link.make_inertial_dict(root, msg)
    assert msg_j == msg and msg_l == msg
    (tmp_path / 'urdf').mkdir()
    Write.write_urdf_sim(joints_dict, {}, inertial_dict, 'arm_description', 'arm',
                         str(tmp_path), fixed_base)
    return ET.parse(tmp_path / 'urdf' / 'arm.xacro').getroot()


def test_world_link_is_root_when_fixed(tmp_path):
    robot = export(tmp_path, fixed_base=True)
    links = {l.get('name') for l in robot.iter('link')}
    children = {j.find('child').get('link') for j in robot.iter('joint')}
    assert links - children == {'world'}
    world_fixed = [j for j in robot.iter('joint') if j.get('name') == 'world_fixed']
    assert len(world_fixed) == 1
    assert world_fixed[0].get('type') == 'fixed'
    assert world_fixed[0].find('parent').get('link') == 'world'
    assert world_fixed[0].find('child').get('link') == 'base_link'


def test_no_world_link_when_not_fixed(tmp_path):
    robot = export(tmp_path, fixed_base=False)
    links = {l.get('name') for l in robot.iter('link')}
    children = {j.find('child').get('link') for j in robot.iter('joint')}
    assert links - children == {'base_link'}


def test_limits_are_decimals_with_defaults(tmp_path):
    robot = export(tmp_path, fixed_base=True)
    limits = list(robot.iter('limit'))
    assert len(limits) == 1
    for attr in ('lower', 'upper', 'effort', 'velocity'):
        value = limits[0].get(attr)
        float(value)
        assert not re.fullmatch(r'-?\d+', value), f'{attr}="{value}" would load as an integer'
    assert float(limits[0].get('velocity')) == Joint.DEFAULT_VELOCITY
    # the Setup Assistant writes whole-number velocities as YAML integers
    assert Joint.DEFAULT_VELOCITY % 1 != 0
    assert float(limits[0].get('effort')) == Joint.DEFAULT_EFFORT
    assert float(limits[0].get('upper')) == pytest.approx(math.pi / 2, abs=1e-6)


def test_limits_from_user_parameters(tmp_path):
    robot = export(tmp_path, fixed_base=True, params={'joint1_velocity': 2.5, 'joint1_effort': 3})
    limit = next(robot.iter('limit'))
    assert limit.get('velocity') == '2.5'
    assert limit.get('effort') == '3.0'


def test_small_inertia_is_not_rounded_to_zero(tmp_path):
    robot = export(tmp_path, fixed_base=True)
    link1 = next(l for l in robot.iter('link') if l.get('name') == 'link1')
    inertia = link1.find('inertial/inertia')
    for attr in ('ixx', 'iyy', 'izz'):
        assert float(inertia.get(attr)) == pytest.approx(1e-8)


def test_link_names(tmp_path):
    robot = export(tmp_path, fixed_base=True)
    links = {l.get('name') for l in robot.iter('link')}
    assert links == {'world', 'base_link', 'link1', 'wheel_1', 'wheel_2'}
    meshes = {m.get('filename').rsplit('/', 1)[1] for m in robot.iter('mesh')}
    assert meshes == {'base_link.stl', 'link1.stl', 'wheel_1.stl', 'wheel_2.stl'}


@pytest.mark.parametrize('design_name, expected', [
    ('example_arm', 'example_arm'),
    ('Example Arm v3', 'example_arm'),
    ('My-Robot (copy)', 'my_robot_copy'),
    ('3dof arm', 'robot_3dof_arm'),
    ('', 'robot'),
])
def test_robot_name_from(design_name, expected):
    assert utils.robot_name_from(design_name) == expected
