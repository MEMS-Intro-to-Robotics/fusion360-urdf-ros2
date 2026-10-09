# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom
from shutil import copytree
import fileinput
import sys

def link_name(occ):
    """
    URDF link name (and mesh file name) for a top-level occurrence.

    A component named base_link gives base_link. A component used once gives
    its own name (link1:1 -> link1). A component used more than once gives the
    occurrence name with ' :()' replaced by '_' (wheel:2 -> wheel_2).
    """
    comp = occ.component
    if comp.name == 'base_link':
        return 'base_link'
    root = comp.parentDesign.rootComponent
    if root.allOccurrencesByComponent(comp).count == 1:
        return re.sub('[ :()]', '_', comp.name)
    return re.sub('[ :()]', '_', occ.name)


def robot_name_from(design_name):
    """
    Default robot name from the design name: drop a trailing version
    ('My Arm v3' -> 'My Arm'), lowercase, and keep [a-z0-9_].
    """
    name = re.sub(r'\s+v\d+$', '', design_name.strip()).lower()
    name = re.sub('[^a-z0-9_]+', '_', name).strip('_')
    if not name:
        name = 'robot'
    elif not name[0].isalpha():
        name = 'robot_' + name
    return name


def copy_occs(root):
    """
    duplicate all the components

    Returns
    ----------
    (new occurrences, [(renamed component, original name)]), for restore_occs
    """
    new_occs_list = []
    renamed = []

    def copy_body(allOccs, occs, name):
        """
        copy the old occs to new component
        """

        bodies = occs.bRepBodies
        transform = adsk.core.Matrix3D.create()

        # Create new components from occs
        # This support even when a component has some occses.

        # free the name first: the new component may take the old component's name
        if occs.component.name != 'old_component':
            renamed.append((occs.component, occs.component.name))
            occs.component.name = 'old_component'
        new_occs = allOccs.addNewComponent(transform)  # this create new occs
        new_occs.component.name = name
        new_occs_list.append(new_occs)
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)

    allOccs = root.occurrences
    # names are taken before any renaming, because link_name reads component names
    coppy_list = [(occs, link_name(occs)) for occs in allOccs]
    for occs, name in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs, name)

    return new_occs_list, renamed


def restore_occs(copied):
    """
    undo copy_occs: delete the copied components and give the original
    components their names back, so the design is left as it was
    """
    new_occs_list, renamed = copied
    for occs in new_occs_list:
        # rename first so the original names are free even if Fusion keeps the
        # deleted component around
        occs.component.name = 'urdf_export_copy'
        occs.deleteMe()
    for component, name in renamed:
        component.name = name


def export_stl(design, save_dir, components):
    """
    export stl files into "sace_dir/"


    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    components: design.allComponents
    """

    # create a single exportManager instance
    exportMgr = design.exportManager
    # get the script location
    try: os.mkdir(save_dir + '/meshes')
    except: pass
    scriptDir = save_dir + '/meshes'
    # export the occurrence one by one in the component to a specified file
    for component in components:
        allOccus = component.allOccurrences
        for occ in allOccus:
            if 'old_component' not in occ.component.name:
                try:
                    print(occ.component.name)
                    fileName = scriptDir + "/" + occ.component.name
                    # create stl exportOptions
                    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                    stlExportOptions.sendToPrintUtility = False
                    stlExportOptions.isBinaryFormat = True
                    # options are .MeshRefinementLow .MeshRefinementMedium .MeshRefinementHigh
                    stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    exportMgr.execute(stlExportOptions)
                except:
                    print('Component ' + occ.component.name + 'has something wrong.')


def file_dialog(ui):
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog'

    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into
    that about center of mass coordinate


    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]


    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2+z**2, x**2+z**2, x**2+y**2,
                         -x*y, -y*z, -x*z]
    return [i - mass*t for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element


    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def create_package(package_name, save_dir, package_dir):
    try: os.mkdir(save_dir + '/launch')
    except: pass

    try: os.mkdir(save_dir + '/urdf')
    except: pass

    try: os.mkdir(save_dir + '/config')
    except: pass

    try: os.mkdir(save_dir + '/' +package_name)
    except: pass
    with open(os.path.join(save_dir, package_name, '__init__.py'), 'w'):
        pass

    try: os.mkdir(save_dir + '/resource')
    except: pass
    with open(os.path.join(save_dir, 'resource', package_name), 'w'):
        pass

    try: os.mkdir(save_dir + '/test')
    except: pass

    copytree(package_dir, save_dir, dirs_exist_ok=True)

def update_setup_py(save_dir, package_name):
    file_name = save_dir + '/setup.py'

    for line in fileinput.input(file_name, inplace=True):
        if "package_name = 'fusion2urdf_ros2'" in line:
            sys.stdout.write("package_name = '" + package_name + "'\n")
        else:
            sys.stdout.write(line)

def update_setup_cfg(save_dir, package_name):
    file_name = save_dir + '/setup.cfg'

    for line in fileinput.input(file_name, inplace=True):
        if "script-dir" in line:
            sys.stdout.write("script-dir=$base/lib/" + package_name + "\n")
        elif "install-scripts" in line:
            sys.stdout.write("install-scripts=$base/lib/" + package_name + "\n")
        else:
            sys.stdout.write(line)

def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("<name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)
