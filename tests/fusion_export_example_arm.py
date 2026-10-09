"""Fusion script: build the course example arm in a new document, export it
without the exporter's dialogs, then close that document without saving.

Run it from Fusion (Scripts and Add-Ins) or through the Fusion MCP script
runner. Set BUILD to the course arm build script and OUT to an output folder.
The calls follow the Gazebo Harmonic branch of Fusion_URDF_Exporter_ROS2.run().
"""
import os
import sys

import adsk.core
import adsk.fusion

EXP = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) if '__file__' in globals() \
    else r'D:\git_repos\fusion360-urdf-ros2'
BUILD = r'D:\git_repos\intro-to-robotics-labs\examples\challenge_hw\cad\build_example_arm_fusion.py'
OUT = os.path.join(os.path.expanduser('~'), 'fusion_urdf_export_test')
ROBOT_NAME = 'example_arm'
FIXED_BASE = True


def snapshot(design):
    """Component names, occurrence names, body counts, and timeline length."""
    root = design.rootComponent
    timeline = design.timeline.count if design.designType == adsk.fusion.DesignTypes.ParametricDesignType else None
    return (sorted(c.name for c in design.allComponents),
            [(o.name, o.bRepBodies.count) for o in root.occurrences],
            timeline)


def run(context):
    app = adsk.core.Application.get()
    previous = app.activeDocument
    ns = {}
    exec(compile(open(BUILD, encoding='utf-8').read(), BUILD, 'exec'), ns)
    ns['run'](context)  # opens a new unsaved document with the arm
    doc = app.activeDocument
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        sys.path.insert(0, EXP)
        from Fusion_URDF_Exporter_ROS2.utils import utils
        from Fusion_URDF_Exporter_ROS2.core import Link, Joint, Write

        package_name = ROBOT_NAME + '_description'
        save_dir = OUT + '/' + package_name
        os.makedirs(save_dir, exist_ok=True)
        package_dir = os.path.join(EXP, 'Fusion_URDF_Exporter_ROS2', 'package') + '/'
        msg = 'Successfully created URDF file'
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        assert msg == 'Successfully created URDF file', msg
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        Write.write_urdf_sim(joints_dict, {}, inertial_dict, package_name, ROBOT_NAME, save_dir, FIXED_BASE)
        Write.write_materials_xacro(joints_dict, {}, inertial_dict, package_name, ROBOT_NAME, save_dir)
        Write.write_ros2control_xacro(joints_dict, {}, inertial_dict, package_name, ROBOT_NAME, save_dir)
        Write.write_gazebo_sim_xacro(joints_dict, {}, inertial_dict, package_name, ROBOT_NAME, save_dir)
        Write.write_display_launch(package_name, ROBOT_NAME, save_dir)
        Write.write_gazebo_sim_launch(package_name, ROBOT_NAME, save_dir)
        utils.create_package(package_name, save_dir, package_dir)
        utils.update_setup_py(save_dir, package_name)
        utils.update_setup_cfg(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)
        before = snapshot(design)
        copied = utils.copy_occs(root)
        try:
            utils.export_stl(design, save_dir, design.allComponents)
        finally:
            utils.restore_occs(copied)
        after = snapshot(design)
        print('exported to ' + save_dir)
        print('design unchanged by export: %s' % (before == after))
        if before != after:
            print('before', before)
            print('after ', after)
    finally:
        # close only the document this script created
        if doc is not previous and not doc.isSaved:
            doc.close(False)
        if previous and previous.isValid:
            previous.activate()
