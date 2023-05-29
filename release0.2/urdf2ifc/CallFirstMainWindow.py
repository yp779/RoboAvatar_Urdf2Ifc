#!/usr/bin/env python

"""
URDF files to ifc converter.
"""

# System
import sys, os

# Check version of Python
if sys.version_info < (3,7):
    sys.exit('This converter requires Python 3.7 or higher.')


# basic lib
import os
import time
import uuid
import argparse
import tempfile


# Math and Eigen
import math
import numpy as np
import scipy.linalg as la


import urdfpy


# Graphic
# ifcopenshell,  urdfpy, OCC libary
import ifcopenshell
from ifcopenshell.geom import main as ifctool


from OCC.Extend.DataExchange import read_stl_file, read_step_file
from OCC.Core.BRepBuilderAPI import (BRepBuilderAPI_GTransform)
from OCC.Core.gp import ( gp_Pnt, gp_Trsf, gp_GTrsf)


# Graphic
# convert dae/obj to stl
# 3.7 <= python version <= 3.11
import aspose.threed as a3d


# error tracking of python
import faulthandler
faulthandler.enable()


# # GUI
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QMessageBox
from PyQt5.QtCore import QTimer, QThread, QMutex
from Ui_FirstMainWindow import *

# python版本为 python=3.11 时候，需要对库进行更改

# 更改
# from collections import Mapping
# from collections.abc import Mapping
# 更改
# from fractions import gcd
# from math import gcd


############################################
# tool functions
############################################

# creat guid
create_guid = lambda: ifcopenshell.guid.compress(uuid.uuid1().hex)


# axis defines
O = 0., 0., 0.
X = 1., 0., 0.
Y = 0., 1., 0.
Z = 0., 0., 1.

# Dir X, Y, Z
dir_0= (0., 0., 0.)
dir_X = (1., 0., 0.)
dir_Y = (0., 1., 0.)
dir_Z = (0., 0., 1.)

# Negative Dir -X, -Y, -Z
dir_nX = (-1., 0., 0.)
dir_nY = (0., -1., 0.)
dir_nZ = (0., 0., -1.)


# dictionary from rpyNum to Coordinate system
Dict_rpyNum_coordinate = {   
    (0, 0, 0):(dir_Z, dir_X),
    (0, 0, 1):(dir_Z, dir_nY),
    (0, 0, 2):(dir_Z, dir_nX),
    (0, 1, 0):(dir_X, dir_nZ), 
    (0, 1, 1):(dir_X, dir_nY),
    (0, 1, 2):(dir_X, dir_Z),
    (0, 2, 0):(dir_nZ, dir_nX),
    (0, 2, 1):(dir_nZ, dir_nY),
    (0, 2, 2):(dir_nZ, dir_X),

    (1, 0, 0):(dir_nY, dir_X),
    (1, 0, 1):(dir_nY, dir_nZ),
    (1, 0, 2):(dir_nY, dir_nX),
    (1, 1, 0):(dir_X, dir_Y),
    (1, 1, 1):(dir_X, dir_nZ),
    (1, 1, 2):(dir_X, dir_nY),
    (1, 2, 0):(dir_Y, dir_nX),
    (1, 2, 1):(dir_Y, dir_nZ),
    (1, 2, 2):(dir_Y, dir_X),

    (2, 0, 0):(dir_nZ, dir_X),
    (2, 0, 1):(dir_nZ, dir_Y),
    (2, 0, 2):(dir_nZ, dir_nX),
    (2, 1, 0):(dir_X, dir_Z),
    (2, 1, 1):(dir_X, dir_Y),
    (2, 1, 2):(dir_X, dir_nZ),
    (2, 2, 0):(dir_Z, dir_nX),
    (2, 2, 1):(dir_Z, dir_Y),
    (2, 2, 2):(dir_Z, dir_X),

    # special cases for 270 degree or -90 degree
    (-1, 0 ,0):(dir_Y, dir_X),
    (0, -1 ,0):(dir_nX, dir_Z),
    (0, 0 ,-1):(dir_nZ, dir_Y),

    (0, 1 ,-1):(dir_X, dir_Y),
    (0, -1, 1):(dir_nX, dir_nY),
    (1, 0 ,-1):(dir_nY, dir_Z),
    (1, -1 ,0):(dir_nX, dir_nY),
    (-1, 1 ,0):(dir_X, dir_nY),
    (-1, 0 ,1):(dir_Y, dir_Z),

    # 3 = -1 here
    (3, 0 ,0):(dir_Y, dir_X),
    (0, 3 ,0):(dir_nX, dir_Z),
    (0, 0 ,3):(dir_nZ, dir_Y),

    (0, 1, 3):(dir_X, dir_Y),
    (0, 3, 1):(dir_nX, dir_nY),
    (1, 0 ,3):(dir_nY, dir_Z),
    (1, 3 ,0):(dir_nX, dir_nY),
    (3, 1 ,0):(dir_X, dir_nY),
    (3, 0 ,1):(dir_Y, dir_Z),
}    


def sprintf(s, fs, *args):
    global sss
    sss = fs % args
    

def log_print(*objects, sep=' ', end='\n', file=None):
    # print("当前时间：" + datetime.now().strftime('%Y-%m-%d %H:%M:%S')) # 这样每次调用log_print()的时候，会先输出当前时间，然后再输出内容
    # print(*objects, sep=' ', end='\n', file=sys.stdout, flush=False)
    print(*objects, sep=' ', end='\n', file=file)
    global myWin
    myWin.textEdit_terminal.append(*objects)
    myWin.textEdit_terminal.repaint()



# 坐标值近似性判断
# 因为ifc中一般以90分度表示，但urdf中的rpy是弧度制，需要做近似
def rpy_to_rightAngle(rad):
        
    if rad >= 0:
        angle_abs = math.degrees(rad)
        revolve_rightAngle_times = angle_abs // 90      # revolve90为旋转90度的数量
        revolve_acuteAngle_degree = angle_abs - (revolve_rightAngle_times*90)
        if revolve_acuteAngle_degree <= 45 :
            revolve_rightAngle_times = revolve_rightAngle_times + 0
        elif revolve_acuteAngle_degree > 45 :
            revolve_rightAngle_times = revolve_rightAngle_times + 1
        return int(revolve_rightAngle_times)

    elif rad < 0:
        angle_abs = math.degrees(-rad)
        revolve_rightAngle_times = angle_abs // 90      # revolve90为旋转90度的数量
        revolve_acuteAngle_degree = angle_abs - (revolve_rightAngle_times*90)
        if revolve_acuteAngle_degree <= 45 :
            revolve_rightAngle_times = revolve_rightAngle_times + 0
        elif revolve_acuteAngle_degree > 45 :
            revolve_rightAngle_times = revolve_rightAngle_times + 1
        return int(-revolve_rightAngle_times)

# rpy转换坐标系

def rpy_get_coordinate(rpy):

    revolve = [(math.pi/2.0)*rpy_to_rightAngle(i) for i in rpy[0:3]] # int, 旋转次数
    # print(revolve)

    initial_coordinate = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])

    rotx_a =    [[1.0, 0.0, 0.0],
                [0.0, math.cos(revolve[0]), -math.sin(revolve[0])],
                [0.0, math.sin(revolve[0]), math.cos(revolve[0])]]

    roty_b =    [[math.cos(revolve[1]), 0.0, math.sin(revolve[1])],
                [0.0, 1.0, 0.0],
                [-math.sin(revolve[1]), 0.0, math.cos(revolve[1])]] 
 
    rotz_c =    [[math.cos(revolve[2]), -math.sin(revolve[2]), 0.0],
                [math.sin(revolve[2]), math.cos(revolve[2]), 0.0],
                [0.0, 0.0, 1.0]]

    array_rotx_a = np.array(rotx_a)
    array_roty_b = np.array(roty_b)
    array_rotz_c = np.array(rotz_c)

    # rpy rotate
    final_coordinate = initial_coordinate @ array_rotx_a
    final_coordinate = final_coordinate @ array_roty_b
    final_coordinate = final_coordinate @ array_rotz_c

    final_rpy = urdfpy.matrix_to_rpy(final_coordinate)
    final_rpyNum = [(int(0) if abs(int(i))<float(0.0001) else int(i/(math.pi/2.0))) for i in final_rpy[0:3]] # int, 旋转次数
    # print(final_rpyNum)

    return final_rpyNum


# 备选方法？？？

# def rpy_get_coordinate(rpy):
#     print("rpy=", end=" ")
#     print(rpy)
#     final_rpyNum = [rpy_to_rightAngle(i) for i in rpy[0:3]] # int, 旋转次数
#     print("rpyNum=", end=" ")
#     print(final_rpyNum)
#     return final_rpyNum


# get dir1 dir2 for ifclocalplacement from rpy
def rpy_get_dir(therpy):
    theDir2_x = math.cos(therpy[1])*math.cos(therpy[2])
    theDir2_y = math.cos(therpy[1])*math.sin(therpy[2])
    theDir2_z = -math.sin(therpy[1])
    theDir2 = [theDir2_x, theDir2_y, theDir2_z]
    
    theDir1_x = math.cos(therpy[0])*math.sin(therpy[1])*math.cos(therpy[2]) + math.sin(therpy[0])*math.sin(therpy[2])
    theDir1_y = math.cos(therpy[0])*math.sin(therpy[1])*math.sin(therpy[2]) - math.sin(therpy[0])*math.cos(therpy[2])
    theDir1_z = math.cos(therpy[0])*math.cos(therpy[1])
    theDir1 = [theDir1_x, theDir1_y, theDir1_z]
    return theDir1, theDir2   
    
def rpy_get_dir_copy(therpy):
    try: 
        rpy_num = tuple(rpy_get_coordinate(therpy))
        coordinate_system_dirs = Dict_rpyNum_coordinate[rpy_num]
        theDir1 = coordinate_system_dirs[0]
        theDir2 = coordinate_system_dirs[1]
    except(KeyError):
        log_print("URDF Rotation Error")
        theDir1 = 0., 0., 0
        theDir2 = 0., 0., 0   
    # print(theDir1, theDir2) 
    return theDir1, theDir2       


# convert Urdf color to ifc color

    
def colorConverter(link):
    color = [link.visuals[0].material.color[0], link.visuals[0].material.color[1], link.visuals[0].material.color[2]]
    return color

# revised for ur5
# def colorConverter(link):
#     if  link.visuals[0].material.color != None:
#         color = [link.visuals[0].material.color[0], link.visuals[0].material.color[1], link.visuals[0].material.color[2]]
#         return color
#     else:
#         color = [0, 0, 0]
#         return color


# Check Urdf path exists with </robot> tag
def checkUrdfPath(inputPath=None):
    validPath = None  
    if not inputPath:
        print("""--input not specified, a URDF content will be read with </robot> tag to stop the reading.""")
    else:
        if not os.path.isfile(inputPath):               #input is not a file
            sys.exit('Input file "%s" does not exists!' % inputPath)
        if not inputPath.endswith('.urdf'):             #input is not a urdf file
            sys.exit('"%s" is not a URDF file!' % inputPath)

        with open(inputPath, 'r') as file:              #open input file with 'r'(read only)
            urdfContent = file.read()
        if urdfContent is None:
            sys.exit('Could not read the URDF file!')   #could not read

        validPath = os.path.abspath(inputPath) #full name of path, abspath = dirname + basename
        return(validPath)


# link_name is a string, from string to find corresponding link
def giveLinkClassFromName(link_name, theRobot):
    # link_name is a string, from string to find corresponding link
    for link in theRobot.links:
        if link.name == link_name:
            return link
        else:            
            pass
            # print("joints relationship error!")  #link_name given in joints does exists in links


# convert mesh to stl format， support suffix of (.stl .dae .obj)
# .stp/.step is supported by read_step_file method, but not supported by this urdf parser  
def mesh_converter(the_mesh_path):

    mesh_path_suffix = os.path.splitext(os.path.basename(the_mesh_path))[-1]

    if mesh_path_suffix == '.stl':
        m_stlShape = read_stl_file(the_mesh_path)
    elif mesh_path_suffix == '.dae':
        # stlOfDae = a3d.Scene.from_file("/home/ilab/URDF2IFC/Project/urdf_models/test.dae")
        stlOfDae = a3d.Scene.from_file(the_mesh_path)
        path_stlOfDae = os.path.dirname(the_mesh_path)+"tempStlFileForDae.stl"
        # print(path_stlOfDae)
        stlOfDae.save(path_stlOfDae)
        m_stlShape = read_stl_file(path_stlOfDae)
        os.remove(path_stlOfDae)
    elif mesh_path_suffix == '.obj':
        # stlOfObj = a3d.Scene.from_file("/home/ilab/URDF2IFC/Project/urdf_models/airboat.obj")
        stlOfObj = a3d.Scene.from_file(the_mesh_path)
        path_stlOfObj = os.path.dirname(the_mesh_path)+"tempStlFileForObj.stl"
        # print(path_stlOfObj)
        stlOfObj.save(path_stlOfObj)
        m_stlShape = read_stl_file(path_stlOfObj)
        os.remove(path_stlOfObj)
    else :
        log_print('%s not support' % mesh_path_suffix)
        m_stlShape = None

    return m_stlShape


# rescaled read-in mesh with scale given in urdf and scale given in ternimal
def mesh_get_rescaled_shape(mesh_path, thescale):
    mesh_shape = mesh_converter(mesh_path)      # Get mesh from .stl or .dae
    trf = gp_Trsf()
    # trf.SetTranslation(gp_Vec(m_scale, m_scale, m_scale))
    trf.SetScale(gp_Pnt(0, 0, 0), thescale)
    gtrf = gp_GTrsf()
    gtrf.SetTrsf(trf)
    mesh_shape_rescaled = BRepBuilderAPI_GTransform(mesh_shape, gtrf, True)

    return mesh_shape_rescaled

############################################
# tool functions
############################################



############################################
# ifc creation related fuctions
############################################

# 放置位置
def create_ifclocalplacement(ifcfile, point=O, dir1=Z, dir2=X, relative_to=None): # Z,X
    axis2placement = create_ifcaxis2placement(ifcfile, point, dir1, dir2)
    ifclocalplacement2 = ifcfile.createIfcLocalPlacement(PlacementRelTo = relative_to, 
                                                         RelativePlacement = axis2placement)
    return ifclocalplacement2

# 2D位置
def create_ifcaxis2placement(ifcfile, point=(.0 , .0, .0), dir1=Z, dir2=X):
    point = ifcfile.createIfcCartesianPoint(point)
    dir1 = ifcfile.createIfcDirection(dir1)
    dir2 = ifcfile.createIfcDirection(dir2)
    axis2placement = ifcfile.createIfcAxis2Placement3D(point, dir1, dir2)
    return axis2placement

# 1D位置
# def create_ifcaxis1placement(ifcfile, point=O, dir1=Z):
#     point = ifcfile.createIfcCartesianPoint(point)
#     dir1 = ifcfile.createIfcDirection(dir1)
#     axis1placement = ifcfile.createIfcAxis1Placement(point, dir1)
#     return axis1placement

# 多边形
def create_ifcpolyline(ifcfile, point_list):
    ifcpts = []
    for point in point_list:
        point = ifcfile.createIfcCartesianPoint(point)
        ifcpts.append(point)
    polyline = ifcfile.createIfcPolyLine(ifcpts)
    return polyline

# 多边形拉伸体
def create_ifcextrudedareasolid(ifcfile, point_list, ifcaxis2placement, extrude_dir, extrusion):
    polyline = create_ifcpolyline(ifcfile, point_list)
    ifcclosedprofile = ifcfile.createIfcArbitraryClosedProfileDef("AREA", None, polyline)
    ifcdir = ifcfile.createIfcDirection(extrude_dir)
    ifcextrudedareasolid = ifcfile.createIfcExtrudedAreaSolid(ifcclosedprofile, ifcaxis2placement, ifcdir, extrusion)
    return ifcextrudedareasolid    

# 圆形
def create_ifccircle(ifcfile, ifcaxis2placement, radius):
    theCircle = ifcfile.createIfcCircle(ifcaxis2placement, radius)
    return theCircle

# 圆形拉伸体（圆柱）
def create_ifcextrudedareasolid_circle(ifcfile, radius, ifcaxis2placement, extrude_dir, extrusion):
    circle = create_ifccircle(ifcfile, ifcaxis2placement, radius)   # radius is folat
    ifcclosedprofile = ifcfile.createIfcArbitraryClosedProfileDef("AREA", None, circle)
    ifcdir = ifcfile.createIfcDirection(extrude_dir)
    ifcextrudedareasolid = ifcfile.createIfcExtrudedAreaSolid(ifcclosedprofile, ifcaxis2placement, ifcdir, extrusion)
    return ifcextrudedareasolid  

# 圆形旋转（圆球）
def create_ifcextrudedareasolid_ball(ifcfile, radius, ifcaxis2placement): # radius is folat
    ifcextrudedareasolid = ifcfile.createIfcSphere(ifcaxis2placement, radius)
    return ifcextrudedareasolid  
                

# 颜色
def colour(ifcfile, entities, color):
    if color is not None:
        r, g, b = color[0], color[1], color[2]
        owner_history = ifcfile.by_type("IfcOwnerHistory")[0]
        context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]
        material = ifcfile.createIfcMaterial("my material")
        material_layer = ifcfile.createIfcMaterialLayer(material, 0.2, None)
        material_layer_set = ifcfile.createIfcMaterialLayerSet([material_layer], None)
        material_layer_set_usage = ifcfile.createIfcMaterialLayerSetUsage(material_layer_set, "AXIS2", "POSITIVE", -0.1)
        ifcfile.createIfcRelAssociatesMaterial(ifcopenshell.guid.new(), owner_history, RelatedObjects=entities,
                                               RelatingMaterial=material_layer_set_usage)
        # 0.93, 0.75, 0.38
        rgb = ifcfile.createIfcColourRgb("my color", r, g, b)
        factor = ifcfile.createIfcNormalisedRatioMeasure(0.65)
        factor1 = ifcfile.createIfcNormalisedRatioMeasure(0.67)
        rendering = ifcfile.createIfcSurfaceStyleRendering(rgb, 0.0, factor, None, None, None, factor1, None, "NOTDEFINED")
        style = ifcfile.createIfcSurfaceStyle("my style", "BOTH", [rendering])
        assignment = ifcfile.createIfcPresentationStyleAssignment([style])
        styled_item = ifcfile.createIfcStyledItem(None, [assignment], None)
        sub_context = ifcfile.createIFCGEOMETRICREPRESENTATIONSUBCONTEXT("Body", "Model",
                                                                         None, None, None, None,
                                                                         context, None, "MODEL_VIEW", None)
        styled_representation = ifcfile.createIFCSTYLEDREPRESENTATION(sub_context, None, None, [styled_item])
        ifcfile.createIfcMaterialDefinitionRepresentation(None, None, [styled_representation], material)      

############################################
# ifc creation related fuctions
############################################


############################################
# converter body
############################################

# the converter
def urdf2IfcConverter(validUrdfPath, savedPath):

    FileName = os.path.basename(validUrdfPath)[:-5] #delete '.urdf' then output as robotName
    Filedir = os.path.dirname(validUrdfPath)
    # outputIfcFile = os.path.join(Filedir, FileName + '.ifc') # defauly dir
    outputIfcFile = os.path.join(savedPath, FileName + '.ifc')
    log_print('Convert ' + os.path.basename(validUrdfPath) + ' to ' + os.path.basename(outputIfcFile))

    ifcfile = create_ifc_template() #create a ifc template

    robot = urdfpy.URDF.load(os.path.abspath(validUrdfPath))

    global progressBarTotal
    progressBarTotal = linkNumCount(robot) + jointNumCount(robot)  # progressBarTotal = linkNum of robot + jointNum of robot
    robContainer = ifcfile.by_type('IfcBuildingStorey')[0]   # Need to be specify where to put the robot in instance tree
    robot_type, link_type, joint_type = define_rob_Rel_Type(ifcfile)
    ifcfile, ifc_robAssemble = addRobot2IfcTemplate(ifcfile, robContainer, robot_type, robot) # add the robot entity
    ifcfile = addLink2IfcTemplate(ifcfile, robot, link_type, ifc_robAssemble) # add link entities
    ifcfile = addJoint2IfcTemplate(ifcfile, robot, joint_type, ifc_robAssemble) # add joint entities
    
    log_print("final ifc file output")
    ifcfile.write(outputIfcFile)    # write file

# count number of links in a robot loaded
def linkNumCount(robot):
    linkNum = 0
    for link in robot.links:
        linkNum += 1
    return linkNum

# count number of joints in a robot loaded
def jointNumCount(robot):
    jointNum = 0
    for joint in robot.joints:
        jointNum += 1
    return jointNum
    
# calculate robot mass by adding up all links
def robotMassCompute(robot):
    robotMass = 0
    for link in robot.links:
        if link.visuals != []:
            robotMass = robotMass + link.inertial.mass                     
    return robotMass

# Recurrent function to check parent-child relationship
def find_child_by_parent(parents, joint_parent_list, joint_child_list, sort_links, sort_joints, robot):
    link_name_list = [link.name for link in robot.links]
    for parent in parents:
        if parent in joint_parent_list: 
            joint_indices = [index for (index, item) in enumerate(joint_parent_list) if item == parent]    
            for i in joint_indices:
                sort_joints_nms = [joint.name for joint in sort_joints]
                if not robot.joints[i].name in sort_joints_nms:                        
                    sort_joints.append(robot.joints[i])            
            childs = [joint_child_list[i] for i in joint_indices]
            for child in childs:
                link_index = link_name_list.index(child)
                sort_links_nms = [link.name for link in sort_links]
                if not child in sort_links_nms:
                    sort_links.append(robot.links[link_index])
            find_child_by_parent(childs, joint_parent_list, joint_child_list, sort_links, sort_joints, robot)

# Re-order the links and joint according to parent-child hierachy
def sort_rob_links_joints(robot):
    sort_links = []
    sort_joints = []
    joint_child_list = [joint.child for joint in robot.joints]
    joint_parent_list = [joint.parent for joint in robot.joints]
    # Find root link
    for link in robot.links:
        if not link.name in joint_child_list:
            sort_links.append(link)
            find_child_by_parent([link.name], joint_parent_list, joint_child_list, sort_links, sort_joints, robot)
            return sort_links, sort_joints

# add the robot entity as a IfcELementAssembly and define its properties
def addRobot2IfcTemplate(ifcfile, robContainer, robot_type, robot):
    owner_history = ifcfile.by_type("IfcOwnerHistory")[0]
    rob_nm = robot.name  
    robot_assembly = ifcfile.createIfcElementAssembly(GlobalId=create_guid(),
                                                      OwnerHistory=owner_history,
                                                      Name=rob_nm,
                                                      Description="This is a robot ifc description",
                                                      PredefinedType="USERDEFINED",
                                                      AssemblyPlace="FACTORY")  
    ifcfile.createIfcRelAggregates(GlobalId=create_guid(),
                                   OwnerHistory=owner_history,                                   
                                   RelatingObject=robContainer,
                                   RelatedObjects=[robot_assembly])  
    ifcfile.createIfcRelDefinesByType(GlobalId=create_guid(),
                                      OwnerHistory=owner_history,                         
                                      RelatingType=robot_type,
                                      RelatedObjects=[robot_assembly])                                       
    robot_PProp_single_nm_list = ["ThroughPut", "SuccessRate", "NaviSpeed", "BatteryLimit"]
    robot_PProp_single_val_type_list = ["IfcText", "IfcText", "IfcText", "IfcText"]
    robot_PProp_single_val_list = ["None", "None", "None", "None"]
    
    robot_MProp_single_nm_list = ["Mass", "NumOfParts"]
    robot_MProp_single_val_type_list = ["IfcText", "IfcText"]
    mass = robotMassCompute(robot)
    NumOfParts = linkNumCount(robot)
    robot_MProp_single_val_list = [str(mass), str(NumOfParts)]   
    
    robot_CProp_single_nm_list = ["Sensing", "Grasping", "Climbing", "PickAndPlace"]
    robot_CProp_single_val_type_list = ["IfcText", "IfcText", "IfcText", "IfcText"]
    robot_CProp_single_val_list = ["TRUE", "TRUE", "FALSE", "TRUE"]  
    
    robot_PProp_val_pairs = []
    for i in range(len(robot_PProp_single_nm_list)):
        prop_nm = robot_PProp_single_nm_list[i]
        prop_val_type = robot_PProp_single_val_type_list[i]
        prop_val = robot_PProp_single_val_list[i]
        robotSinProp = create_IfcPropertySingleValue(ifcfile, prop_nm, None, prop_val_type, prop_val, None)
        robot_PProp_val_pairs.append(robotSinProp)
    
    robot_MProp_val_pairs = []
    for i in range(len(robot_MProp_single_nm_list)):
        prop_nm = robot_MProp_single_nm_list[i]
        prop_val_type = robot_MProp_single_val_type_list[i]
        prop_val = robot_MProp_single_val_list[i]
        robotSinProp = create_IfcPropertySingleValue(ifcfile, prop_nm, None, prop_val_type, prop_val, None)
        robot_MProp_val_pairs.append(robotSinProp)
    
    robot_CProp_val_pairs = []
    for i in range(len(robot_CProp_single_nm_list)):
        prop_nm = robot_CProp_single_nm_list[i]
        prop_val_type = robot_CProp_single_val_type_list[i]
        prop_val = robot_CProp_single_val_list[i]
        robotSinProp = create_IfcPropertySingleValue(ifcfile, prop_nm, None, prop_val_type, prop_val, None)
        robot_CProp_val_pairs.append(robotSinProp)
    
    listProperpertyEnumNm = get_list_of_IfcPropertyEnumeration(ifcfile)
    RobotLocomotionEnum = None
    if "RobotLocomotionEnum" in listProperpertyEnumNm:
        ind = listProperpertyEnumNm.index("RobotLocomotionEnum")
        RobotLocomotionEnum = ifcfile.by_type("IfcPropertyEnumeration")[ind]
    else:
        propEnumNm = "RobotLocomotionEnum"
        EnumValList = ["Wheel","Leg","WheelLeg","Fly","Fixed"]
        RobotLocomotionEnum = create_IfcPropertyEnumeration(ifcfile, propEnumNm, "IfcLabel", EnumValList, None)
    robLocoType = "Fixed" # Need to manually specify as appropriate
    robot_CProp_val_pairs.append(create_IfcPropertyEnumeratedValue(ifcfile, "Locomotion", None, robLocoType, "IfcLabel", RobotLocomotionEnum))    
    define_propset_to_ele(ifcfile, "Robot_ProductivityProps", robot_PProp_val_pairs, robot_assembly)
    define_propset_to_ele(ifcfile, "Robot_MechanicalProps", robot_MProp_val_pairs, robot_assembly)
    define_propset_to_ele(ifcfile, "Robot_CapabilityProps", robot_CProp_val_pairs, robot_assembly)
    log_print("Robot assembly added")   
    return ifcfile, robot_assembly
                      

# load Urdf file and add link entities to ifc file
def addLink2IfcTemplate(ifcfile, robot, link_type, ifc_robAssemble):
    #robot = urdfpy.URDF.load(validUrdfPath)
    sort_links, sort_joints = sort_rob_links_joints(robot)
    print(sort_links)
    print(sort_joints)
    link_ind = 0
    for link in sort_links:
        # print("This link is " + link.name)
        log_print('*** ' + 'Building link ' + link.name +' ***')
        rob_link = create_link(ifcfile, link, link_ind, robot, ifc_robAssemble)
        if rob_link != None:
            ifcfile.createIfcRelDefinesByType(GlobalId=create_guid(),
                                              OwnerHistory=ifcfile.by_type("IfcOwnerHistory")[0],                         
                                              RelatingType=link_type,
                                              RelatedObjects=[rob_link])
            linkMass = None
            linkInertia = None
            if link.inertial != None:
                linkMass = link.inertial.mass  
                linkInertia = [link.inertial.inertia[0][0],
                               link.inertial.inertia[0][1],
                               link.inertial.inertia[0][2],
                               link.inertial.inertia[1][1],
                               link.inertial.inertia[1][2],
                               link.inertial.inertia[2][2]]            
            define_link_prop_val_pairs(ifcfile, linkMass, linkInertia, rob_link)
            link_ind = link_ind + 1
        myWin.progressBarSet()
    log_print("All links added")   
    return ifcfile


# load Urdf file and add joint entities to ifc file
def addJoint2IfcTemplate(ifcfile, robot, joint_type, ifc_robAssemble):
    sort_links, sort_joints = sort_rob_links_joints(robot)
    for joint in sort_joints:
        log_print('*** ' + 'Building joint ' + joint.name +' ***')
        rob_joint = create_joint(ifcfile, joint, robot, ifc_robAssemble)
        ifcfile.createIfcRelDefinesByType(GlobalId=create_guid(),
                                          OwnerHistory=ifcfile.by_type("IfcOwnerHistory")[0],                         
                                          RelatingType=joint_type,
                                          RelatedObjects=[rob_joint])        
        jointMovType = joint.joint_type
        jointMovAxis = None
        jointMovLimit = None
        if jointMovType in ["revolute", "prismatic"]:
            jointMovAxis = joint.axis
            jointMovLimit = [joint.limit.lower, joint.limit.upper]            
        elif jointMovType in ["continuous", "planar"]:
            jointMovAxis = joint.axis                
        define_joint_prop_val_pairs(ifcfile, jointMovType, jointMovAxis, jointMovLimit, rob_joint)     
        myWin.progressBarSet()
    log_print("All joints added")   
    return ifcfile


# find out relative placement from robot.joints 
def create_link_placement(ifcfile, link_name, link_ind, theRobot):

    theLink = giveLinkClassFromName(link_name, theRobot)

    j_scale = float(scale)

    if link_ind == 0 :  # root of link trees, i.e., base_link or link_footprint
        site_placement = ifcfile.by_type("IfcSite")[0].ObjectPlacement        
        j_extrusion_placement = create_ifclocalplacement(ifcfile, relative_to=site_placement)
        return j_extrusion_placement
    else:   # not base_link                                  
        for joint in theRobot.joints:
            if theLink.name == joint.child :   #joint gives the parent link
                # print("This joint is " + joint.name)
                j_xyz_rpy = urdfpy.matrix_to_xyz_rpy(joint.origin)
                j_xyz = [j_scale*float(i) for i in j_xyz_rpy[0:3]] #float
                j_rpy = j_xyz_rpy[3:6]                
                j_dir1, j_dir2 = rpy_get_dir(j_rpy)               
                log_print('Parent(%s) -> Child(%s)' % (joint.parent, joint.child)) 
                parent_Link_nm = joint.parent
                parent_Link = get_ifc_entity_by_type_name(ifcfile, "IfcBuildingElementProxy", parent_Link_nm)
                parent_link_placement = parent_Link.ObjectPlacement.PlacementRelTo
                child_extrusion_placement = create_ifclocalplacement(ifcfile, point=(j_xyz[0], j_xyz[1], j_xyz[2]), dir1 = j_dir1, dir2 = j_dir2, relative_to = parent_link_placement)
                j_extrusion_placement = child_extrusion_placement
                return j_extrusion_placement
            else:
                pass
                #print("joints relationship error!")    


# judge class of link and then creat(box, cylinder, sphere, mesh)
def create_link(ifcfile, link, link_ind, robot, ifc_robAssemble):
    try:
        if link.visuals == []:
            rob_link = create_nonvisual(ifcfile, link, link_ind, robot, ifc_robAssemble)
            return rob_link         
        elif link.visuals[0].geometry.box != None:
            rob_link = create_box(ifcfile, link, link_ind, robot, ifc_robAssemble)
            return rob_link
        elif link.visuals[0].geometry.cylinder != None:
            rob_link = create_cylinder(ifcfile, link, link_ind, robot, ifc_robAssemble)
            return rob_link
        elif link.visuals[0].geometry.sphere != None:
            rob_link = create_sphere(ifcfile, link, link_ind, robot, ifc_robAssemble)
            return rob_link
        elif link.visuals[0].geometry.mesh != None:        
            rob_link = create_mesh(ifcfile, link, link_ind, robot, ifc_robAssemble)
            return rob_link        
        else:
            pass
    except Exception as e:
        log_print(str(e))
        return None
    # except :
    #     log_print("Other error")        


# create joint entities 
def create_joint(ifcfile, joint, robot, ifc_robAssemble):
    owner_history = ifcfile.by_type("IfcOwnerHistory")[0]
    joint_nm = joint.name
    joint_parent_nm = joint.parent    
    joint_child_nm = joint.child
    joint_des = "This is the joint connecting " + joint_parent_nm + "with " + joint_child_nm
    joint_parent = get_ifc_entity_by_type_name(ifcfile, "IfcBuildingElementProxy", joint_parent_nm)
    joint_child = get_ifc_entity_by_type_name(ifcfile, "IfcBuildingElementProxy", joint_child_nm)
    joint_placement = joint_child.ObjectPlacement.PlacementRelTo
    joint_entity = ifcfile.createIfcVirtualElement(GlobalId=create_guid(),
                                                   OwnerHistory=owner_history,
                                                   Name=joint_nm,
                                                   Description=joint_des,
                                                   ObjectPlacement=joint_placement)  
    ifcfile.createIfcRelAggregates(GlobalId=create_guid(),
                                   OwnerHistory=owner_history,
                                   RelatingObject=ifc_robAssemble,
                                   RelatedObjects=[joint_entity])
    ifcfile.createIfcRelAssignsToProduct(GlobalId=create_guid(),
                                         OwnerHistory=owner_history,
                                         Name="parentRel_" + joint_parent_nm + "_" + joint_nm,
                                         Description="This is relationship assigning " + joint_nm + "to its parent " + joint_parent_nm,
                                         RelatingProduct=joint_parent,
                                         RelatedObjects=[joint_entity])
    ifcfile.createIfcRelAssignsToProduct(GlobalId=create_guid(),
                                         OwnerHistory=owner_history,
                                         Name="childRel_" + joint_nm + "_" + joint_child_nm,
                                         Description="This is relationship assigning " + joint_nm + "to its child " + joint_child_nm,
                                         RelatingProduct=joint_entity,
                                         RelatedObjects=[joint_child])
    return joint_entity

############################################
# converter body
############################################


############################################
# add ifc entities functions
############################################

# Get ifc entity by type and name
def get_ifc_entity_by_type_name(ifcfile, ifcType, name):
    entity_list = ifcfile.by_type(ifcType)
    for entity in entity_list:
        if entity.Name == name:
            return entity
    
# Get a list of property enumeration
def get_list_of_IfcPropertyEnumeration(ifcfile):
    listProperpertyEnumNm = []
    propertyEnumList = ifcfile.by_type("IfcPropertyEnumeration")
    for propertyEnum in propertyEnumList:
        listProperpertyEnumNm.append(propertyEnum.Name)
    return listProperpertyEnumNm

# Create single-value properties
def create_IfcPropertySingleValue(ifcfile, propNm, propDes, propValType, propVal, propUnit):
    ifcValEntity = ifcfile.create_entity(propValType, propVal)
    SingleValProp = ifcfile.createIfcPropertySingleValue(propNm,propDes,ifcValEntity,propUnit)
    return SingleValProp

# Create property enumeration
def create_IfcPropertyEnumeration(ifcfile, propEnumNm, EnumValType, EnumValList, propEnumUnit):
    EnumValEntityList = []
    for EnumVal in EnumValList:
        EnumValEntityList.append(ifcfile.create_entity(EnumValType, EnumVal))   
    EnumEntity = ifcfile.createIfcPropertyEnumeration(propEnumNm, EnumValEntityList, propEnumUnit)
    return EnumEntity
    
# Create enumeration-value properties
def create_IfcPropertyEnumeratedValue(ifcfile, propNm, propDes, propVal, EnumValType, EnumEntity):
    ifcValEntity = ifcfile.create_entity(EnumValType, propVal)
    EnumValProp = ifcfile.createIfcPropertyEnumeratedValue(propNm,propDes,[ifcValEntity],EnumEntity)
    return EnumValProp

# Create list-value properties
def create_IfcPropertyListValue(ifcfile, propNm, propDes, propValType, propValList, propUnit):
    propValListEntity = []
    for propVal in propValList:
        propValListEntity.append(ifcfile.create_entity(propValType, propVal))   
    ListValProp = ifcfile.createIfcPropertyListValue(propNm, propDes, propValListEntity, propUnit)
    return ListValProp

# Define propertyset and relate it to elements
def define_propset_to_ele(ifcfile, propSetNm, ifc_prop_val_pairs, relatedEle):
    ifc_prop_set = ifcfile.createIfcPropertySet(GlobalId=create_guid(),
                                                OwnerHistory=ifcfile.by_type("IfcOwnerHistory")[0],
                                                Name=propSetNm,
                                                HasProperties=ifc_prop_val_pairs)
    ifcfile.createIfcRelDefinesByProperties(GlobalId=create_guid(),
                                            OwnerHistory=ifcfile.by_type("IfcOwnerHistory")[0],                     
                                            RelatingPropertyDefinition=ifc_prop_set,
                                            RelatedObjects=[relatedEle]) 
                                            
# Define typeObjects related to robots, including robotType, linkType, jointType
def define_rob_Rel_Type(ifcfile):
    owner_history = ifcfile.by_type("IfcOwnerHistory")[0]
    robot_type = ifcfile.createIfcTypeObject(GlobalId=create_guid(),
                                             OwnerHistory=owner_history,
                                             Name="RobotType",
                                             Description="This is the type object of robot occurrence",
                                             ApplicableOccurrence="IfcElementAssembly/ROBOT")
    link_type = ifcfile.createIfcTypeObject(GlobalId=create_guid(),
                                            OwnerHistory=owner_history,
                                            Name="LinkType",
                                            Description="This is the type object of robot link occurrence",
                                            ApplicableOccurrence="IfcBuildingElementProxy/LINK")
    joint_type = ifcfile.createIfcTypeObject(GlobalId=create_guid(),
                                             OwnerHistory=owner_history,
                                             Name="JointType",
                                             Description="This is the type object of robot joint occurrence",
                                             ApplicableOccurrence="IfcVirtualElement/JOINT")
    return robot_type, link_type, joint_type

# Define link property set and assign it to corresponding link
def define_link_prop_val_pairs(ifcfile, mass, inertiaMat, rob_link):
    link_MProp_single_nm_list = ["Mass"]
    link_MProp_single_val_type_list = ["IfcText"]
    link_MProp_single_val_list = [str(mass)]
    
    link_CProp_single_nm_list = ["Sensing","Grasping","Climbing","PickAndPlace"]
    link_CProp_single_val_type_list = ["IfcText", "IfcText", "IfcText", "IfcText"]
    link_CProp_single_val_list = ["FLASE", "FALSE", "FALSE", "FALSE"]
    
    ifc_link_MProp_val_pairs = []
    for i in range(len(link_MProp_single_nm_list)):
        prop_single_nm = link_MProp_single_nm_list[i]
        prop_single_val_type = link_MProp_single_val_type_list[i]
        prop_single_val = link_MProp_single_val_list[i]
        ifc_link_MProp_val_pairs.append(ifcfile.createIfcPropertySingleValue(prop_single_nm,None,ifcfile.create_entity(prop_single_val_type, prop_single_val),None))
    
    ifc_link_CProp_val_pairs = []
    for i in range(len(link_CProp_single_nm_list)):
        prop_single_nm = link_CProp_single_nm_list[i]
        prop_single_val_type = link_CProp_single_val_type_list[i]
        prop_single_val = link_CProp_single_val_list[i]
        ifc_link_CProp_val_pairs.append(ifcfile.createIfcPropertySingleValue(prop_single_nm,None,ifcfile.create_entity(prop_single_val_type, prop_single_val),None))
    
    if inertiaMat != None:
        ifc_link_MProp_val_pairs.append(create_IfcPropertyListValue(ifcfile, "InertiaMat", "ixx, ixy, ixz, iyy, iyz, izz", "IfcText", [str(i) for i in inertiaMat], None))
    else:
        ifc_link_MProp_val_pairs.append(ifcfile.createIfcPropertySingleValue("InertiaMat", "ixx, ixy, ixz, iyy, iyz, izz", ifcfile.create_entity("IfcText", "None"),None))
    define_propset_to_ele(ifcfile, "Link_MechanicalProps", ifc_link_MProp_val_pairs, rob_link)
    define_propset_to_ele(ifcfile, "Link_CapabilityProps", ifc_link_CProp_val_pairs, rob_link)

# Define joint property set and assign it to corresponding joint
def define_joint_prop_val_pairs(ifcfile, moveType, moveAxis, moveLimit, rob_joint):
    ifc_joint_prop_val_pairs = []
    # move type
    listProperpertyEnumNm = get_list_of_IfcPropertyEnumeration(ifcfile)
    JointMovTypeEnum = None
    if "JointMovTypeEnum" in listProperpertyEnumNm:
        ind = listProperpertyEnumNm.index("JointMovTypeEnum")
        JointMovTypeEnum = ifcfile.by_type("IfcPropertyEnumeration")[ind]
    else:
        propEnumNm = "JointMovTypeEnum"
        EnumValList = ["revolute","continuous","prismatic","fixed","floating","planar"]
        JointMovTypeEnum = create_IfcPropertyEnumeration(ifcfile, propEnumNm, "IfcLabel", EnumValList, None)
    ifc_joint_prop_val_pairs.append(create_IfcPropertyEnumeratedValue(ifcfile, "JointMovType", None, moveType, "IfcLabel", JointMovTypeEnum))  
    # move axis
    if type(moveAxis) != type(None):
        ifc_joint_prop_val_pairs.append(create_IfcPropertyListValue(ifcfile, "JointMovAxis", "Normalized [x,y,z]: axis of rotation for revolute/continuous, axis of translation for prismatic, surface normal for planar, None for others", "IfcText", [str(i) for i in moveAxis], None))
    else:
        ifc_joint_prop_val_pairs.append(ifcfile.createIfcPropertySingleValue("JointMovAxis", "Normalized [x,y,z]: axis of rotation for revolute/continuous, axis of translation for prismatic, surface normal for planar, None for others", ifcfile.create_entity("IfcText", "None"),None))
    # move limit
    if type(moveLimit) != type(None):
        ifc_joint_prop_val_pairs.append(create_IfcPropertyListValue(ifcfile, "JointMovLimit", "[Lower, Upper]: lower and upper limit of movement, only applied to revolute and prismatic", "IfcText", [str(i) for i in moveLimit], None))
    else:
        ifc_joint_prop_val_pairs.append(ifcfile.createIfcPropertySingleValue("JointMovLimit", "[Lower, Upper]: lower and upper limit of movement, only applied to revolute and prismatic", ifcfile.create_entity("IfcText", "None"),None))
    # assign property to elements
    define_propset_to_ele(ifcfile, "Joint_MechanicTopoProps", ifc_joint_prop_val_pairs, rob_joint)

# Creat Box-shape non-visual root link, e.g., base_footprint for turtlebot
def create_nonvisual(ifcfile, link, link_ind, robot, ifc_robAssemble):

    global scale    
    n_scale = float(scale)

    length = 0.001
    width = 0.001
    height = 0.000002        

    n_robAss = ifc_robAssemble
    n_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    n_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    n_xyz = [0.0, 0.0, 0.0] #float
    n_rpy = [0.0, 0.0, 0.0]
    n_dir1, n_dir2 = rpy_get_dir(n_rpy)

    n_joint_to_base_placement = create_link_placement(ifcfile, link.name, link_ind, robot)
    n_space_placement = create_ifclocalplacement(ifcfile, point=(n_xyz[0], n_xyz[1], n_xyz[2]), dir1 = n_dir1, dir2 = n_dir2, relative_to = n_joint_to_base_placement)
    n_extrusion_placement = create_ifcaxis2placement(ifcfile)

    n_solid = create_ifcextrudedareasolid(ifcfile,
                                        #[(0.0, 0.0, 0.0), (length, 0.0, 0.0), (length, width, 0.0), (0.0, width, 0.0)],
                                        [(-length/2.0, -width/2.0, -height/2.0), (-length/2.0, width/2.0, -height/2.0), (length/2.0, width/2.0, -height/2.0), (length/2.0, -width/2.0, -height/2.0)],
                                        n_extrusion_placement,
                                        (0.0, 0.0, 1.0),
                                        float(height))

    n_body_representation = ifcfile.createIfcShapeRepresentation(n_context, "robotBody", "Box", [n_solid])
    n_entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [n_body_representation])
    n_entity = ifcfile.create_entity('IfcBuildingElementProxy', ifcopenshell.guid.new())
    n_entity.OwnerHistory = n_OwnerHistory
    n_entity.Name = link.name
    n_entity.Representation = n_entity_shape
    n_entity.ObjectPlacement = n_space_placement
    n_relatingObject = n_robAss
    n_related_objects = []
    n_related_objects.append(n_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                n_OwnerHistory,
                                None,
                                None,
                                n_relatingObject,
                                n_related_objects)    
    log_print('*** ' + 'Invisible link Built: %s' % link.name + ' ***\n')
    return n_entity


# Creat Box-shape link
def create_box(ifcfile, link, link_ind, robot, ifc_robAssemble):

    global scale    
    b_scale = float(scale)

    length = float(b_scale*link.visuals[0].geometry.box.size[0])
    width = float(b_scale*link.visuals[0].geometry.box.size[1])
    height = float(b_scale*link.visuals[0].geometry.box.size[2])

    b_color = colorConverter(link)
    #print(b_color)

    b_robAss = ifc_robAssemble
    b_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    b_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    b_xyz_rpy = urdfpy.matrix_to_xyz_rpy(link.visuals[0].origin)
    b_xyz = [b_scale*float(i) for i in b_xyz_rpy[0:3]] #float
    b_rpy = b_xyz_rpy[3:6]

    b_dir1, b_dir2 = rpy_get_dir(b_rpy)
    # print(b_xyz_rpy)
    # print(b_xyz)
    # print(b_rpy)
    # print(b_dir1, b_dir2)

    b_joint_to_base_placement = create_link_placement(ifcfile, link.name, link_ind, robot)
    b_space_placement = create_ifclocalplacement(ifcfile, point=(b_xyz[0], b_xyz[1], b_xyz[2]), dir1 = b_dir1, dir2 = b_dir2, relative_to = b_joint_to_base_placement)
    b_extrusion_placement = create_ifcaxis2placement(ifcfile)

    b_solid = create_ifcextrudedareasolid(ifcfile,
                                        #[(0.0, 0.0, 0.0), (length, 0.0, 0.0), (length, width, 0.0), (0.0, width, 0.0)],
                                        [(-length/2.0, -width/2.0, -height/2.0), (-length/2.0, width/2.0, -height/2.0), (length/2.0, width/2.0, -height/2.0), (length/2.0, -width/2.0, -height/2.0)],
                                        b_extrusion_placement,
                                        (0.0, 0.0, 1.0),
                                        float(height))

    b_body_representation = ifcfile.createIfcShapeRepresentation(b_context, "robotBody", "Box", [b_solid])
    b_entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [b_body_representation])
    b_entity = ifcfile.create_entity('IfcBuildingElementProxy', ifcopenshell.guid.new())
    b_entity.OwnerHistory = b_OwnerHistory
    b_entity.Name = link.name
    b_entity.Representation = b_entity_shape
    b_entity.ObjectPlacement = b_space_placement
    b_relatingObject = b_robAss
    b_related_objects = []
    b_related_objects.append(b_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                b_OwnerHistory,
                                None,
                                None,
                                b_relatingObject,
                                b_related_objects)
    colour(ifcfile, b_related_objects, b_color) 
    log_print('*** ' + 'Box Built: %s' % link.name + ' ***\n')
    return b_entity


# Create Cylinder-shape link
def create_cylinder(ifcfile, link, link_ind, robot, ifc_robAssemble):

    global scale    
    c_scale = float(scale)


    length = float(c_scale*link.visuals[0].geometry.cylinder.length)
    radius = float(c_scale*link.visuals[0].geometry.cylinder.radius)
    
    c_color = colorConverter(link)
    #print(c_color)

    c_robAss = ifc_robAssemble
    c_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    c_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    c_xyz_rpy = urdfpy.matrix_to_xyz_rpy(link.visuals[0].origin)
    c_xyz = [c_scale*float(i) for i in c_xyz_rpy[0:3]] #float
    c_rpy = c_xyz_rpy[3:6]

    c_dir1, c_dir2 = rpy_get_dir(c_rpy)
    # print(c_xyz_rpy)
    # print(c_xyz)
    # print(c_rpy)
    # print(c_dir1, c_dir2)

    c_joint_to_base_placement = create_link_placement(ifcfile, link.name, link_ind, robot)
    c_space_placement = create_ifclocalplacement(ifcfile, point=(c_xyz[0], c_xyz[1], c_xyz[2]), dir1 = c_dir1, dir2 = c_dir2, relative_to=c_joint_to_base_placement)
    c_extrusion_placement = create_ifcaxis2placement(ifcfile, point=(float(0) ,float(0),float(-length/4.0)))

    c_solid = create_ifcextrudedareasolid_circle(ifcfile,
                                                radius,
                                                c_extrusion_placement,
                                                (0.0, 0.0, 1.0),
                                                float(length))

    c_body_representation = ifcfile.createIfcShapeRepresentation(c_context, "robotBody", "Cylinder", [c_solid])
    c_entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [c_body_representation])
    c_entity = ifcfile.create_entity('IfcBuildingElementProxy', ifcopenshell.guid.new())
    c_entity.OwnerHistory = c_OwnerHistory
    c_entity.Name = link.name
    c_entity.Representation = c_entity_shape
    c_entity.ObjectPlacement = c_space_placement
    c_relatingObject = c_robAss
    c_related_objects = []
    c_related_objects.append(c_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                c_OwnerHistory,
                                None,
                                None,
                                c_relatingObject,
                                c_related_objects)
    colour(ifcfile, c_related_objects, c_color) 
    log_print('*** ' + 'Cylinder Built: %s' % link.name + ' ***\n')
    return c_entity

# Create Sphere-shape link
def create_sphere(ifcfile, link, link_ind, robot, ifc_robAssemble):

    global scale    
    s_scale = float(scale)

    radius = float(s_scale*link.visuals[0].geometry.sphere.radius)
    
    s_color = colorConverter(link)
    #print(s_color)

    s_robAss = ifc_robAssemble
    s_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    s_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    s_xyz_rpy = urdfpy.matrix_to_xyz_rpy(link.visuals[0].origin)
    s_xyz = [s_scale*float(i) for i in s_xyz_rpy[0:3]] #float
    s_rpy = s_xyz_rpy[3:6]

    s_dir1, s_dir2 = rpy_get_dir(s_rpy)
    # print(s_xyz_rpy)
    # print(s_xyz)
    # print(s_rpy)
    # print(s_dir1, s_dir2)

    s_joint_to_base_placement = create_link_placement(ifcfile, link.name, link_ind, robot)
    s_space_placement = create_ifclocalplacement(ifcfile, point=(s_xyz[0], s_xyz[1], s_xyz[2]), dir1 = s_dir1, dir2 = s_dir2, relative_to = s_joint_to_base_placement)
    s_sphere_placement = create_ifcaxis2placement(ifcfile)

    s_solid = create_ifcextrudedareasolid_ball(ifcfile,
                                                radius,
                                                s_sphere_placement)

    s_body_representation = ifcfile.createIfcShapeRepresentation(s_context, "robotBody", "Sphere", [s_solid])
    s_entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [s_body_representation])
    s_entity = ifcfile.create_entity('IfcBuildingElementProxy', ifcopenshell.guid.new())
    s_entity.OwnerHistory = s_OwnerHistory
    s_entity.Name = link.name
    s_entity.Representation = s_entity_shape
    s_entity.ObjectPlacement = s_space_placement
    s_relatingObject = s_robAss
    s_related_objects = []
    s_related_objects.append(s_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                s_OwnerHistory,
                                None,
                                None,
                                s_relatingObject,
                                s_related_objects)
    colour(ifcfile, s_related_objects, s_color) 
    log_print('*** ' + 'Sphere Built: %s' % link.name + ' ***\n')
    return s_entity

# Create Mesh-shape link
def create_mesh(ifcfile, link, link_ind, robot, ifc_robAssemble):
    
    global scale   

    mesh_path = link.visuals[0].geometry.mesh.filename

    # 默认mesh各个轴尺度上的scale都相等，若不相等报错
    if link.visuals[0].geometry.mesh.scale[0] != None:
        if link.visuals[0].geometry.mesh.scale[0] == link.visuals[0].geometry.mesh.scale[1] == link.visuals[0].geometry.mesh.scale[2]:
            m_scale = float(scale * link.visuals[0].geometry.mesh.scale[0])
        else:
            m_scale = float(scale)
            log_print("The mesh scale is error, set as 1.0")
    else:
        m_scale = float(scale)
        log_print("No mesh scale is given, set as 1.0")

    #print(m_scale) 
    m_color = colorConverter(link)
    # print(m_color)

    m_robAss = ifc_robAssemble
    m_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    m_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    m_xyz_rpy = urdfpy.matrix_to_xyz_rpy(link.visuals[0].origin)
    m_xyz = [scale*float(i) for i in m_xyz_rpy[0:3]] #float
    m_rpy = m_xyz_rpy[3:6]

    m_dir1, m_dir2 = rpy_get_dir(m_rpy)
    # print(m_xyz_rpy)
    # print(m_xyz)
    # print(m_rpy)
    # print(m_dir1, m_dir2)

    m_joint_to_base_placement = create_link_placement(ifcfile, link.name, link_ind, robot)
    m_space_placement = create_ifclocalplacement(ifcfile, point=(m_xyz[0], m_xyz[1], m_xyz[2]), dir1 = m_dir1, dir2 = m_dir2, relative_to = m_joint_to_base_placement)
    
    # Get mesh from .stl or .dae
    # Rescaled mesh shape
    m_rescaled_shape = mesh_get_rescaled_shape(mesh_path, m_scale)

    m_entity_rescaled_shape = ifctool.tesselate(ifcfile.schema, m_rescaled_shape.Shape(), 1.)
    m_entity_rescaled_shape.Representations[0].ContextOfItems = m_context
    #ifcfile.add(m_entity_shape)
    m_entity = ifcfile.create_entity('IfcBuildingElementProxy', ifcopenshell.guid.new())
    m_entity.OwnerHistory = m_OwnerHistory
    m_entity.Name = link.name
    m_entity.Representation = m_entity_rescaled_shape
    m_entity.ObjectPlacement = m_space_placement
    m_relatingObjects = m_robAss
    m_related_objects = []
    m_related_objects.append(m_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                   m_OwnerHistory,
                                   None,
                                   None,
                                   m_relatingObjects,
                                   m_related_objects)
    colour(ifcfile, m_related_objects, m_color)
    log_print('*** ' + 'Mesh Built: %s' % link.name + ' ***\n')
    return m_entity

############################################
# add ifc entities functions
############################################


############################################
#  create ifc template
############################################

def create_ifc_template():
    
    # Write the template to a temporary file
    ifcTemplate = read_ifc_basic_template()
    temp_handle, temp_filename = tempfile.mkstemp(suffix=".ifc")
    with open(temp_filename, "wb") as f:
        f.write(ifcTemplate.encode())

    theifcfile = ifcopenshell.open(temp_filename)
    owner_history = theifcfile.by_type("IfcOwnerHistory")[0]
    project = theifcfile.by_type("IfcProject")[0]
    context = theifcfile.by_type("IfcGeometricRepresentationContext")[0]
    site_placement = create_ifclocalplacement(theifcfile)
    site = theifcfile.createIfcSite(create_guid(), owner_history, "Site", None, None, site_placement, None, None, "ELEMENT", None, None, None, None, None)
    building_placement = create_ifclocalplacement(theifcfile, relative_to=site_placement)
    building = theifcfile.createIfcBuilding(create_guid(), owner_history, 'Building', None, None, building_placement, None, None, "ELEMENT", None, None, None)
    storey_placement = create_ifclocalplacement(theifcfile, relative_to=building_placement)
    elevation = 0.0
    building_storey = theifcfile.createIfcBuildingStorey(create_guid(), owner_history, 'Storey', None, None, storey_placement, None, None, "ELEMENT", elevation)
    container_storey = theifcfile.createIfcRelAggregates(create_guid(), owner_history, "Building Container", None, building, [building_storey])
    container_site = theifcfile.createIfcRelAggregates(create_guid(), owner_history, "Site Container", None, site, [building])
    container_project = theifcfile.createIfcRelAggregates(create_guid(), owner_history, "Project Container", None, project, [site])
    flow_placement = create_ifclocalplacement(theifcfile, relative_to=storey_placement)

    log_print("ifc template created successfully \n")
    return theifcfile


def read_ifc_basic_template():
    #create_guid = lambda: ifcopenshell.guid.compress(uuid.uuid1().hex)
    
    # IFC template creation
    filename = "demo1.ifc"
    timestamp = round(time.time())
    timestring = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(timestamp))
    creator = "Yipeng Pan"
    organization = "HKU"
    application, application_version = "IfcOpenShell", "0.7"
    project_globalid, project_name = create_guid(), "Urdf2ifc"    

    template = """ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('ViewDefinition [CoordinationView]'),'2;1');
FILE_NAME('%(filename)s','%(timestring)s',('%(creator)s'),('%(organization)s'),'%(application)s','%(application)s','');
FILE_SCHEMA(('IFC4'));
ENDSEC;
DATA;
#1=IFCPERSON($,$,'%(creator)s',$,$,$,$,$);
#2=IFCORGANIZATION($,'%(organization)s',$,$,$);
#3=IFCPERSONANDORGANIZATION(#1,#2,$);
#4=IFCAPPLICATION(#2,'%(application_version)s','%(application)s','');
#5=IFCOWNERHISTORY(#3,#4,$,.ADDED.,$,#3,#4,%(timestamp)s);
#6=IFCDIRECTION((1.,0.,0.));
#7=IFCDIRECTION((0.,0.,1.));
#8=IFCCARTESIANPOINT((0.,0.,0.));
#9=IFCAXIS2PLACEMENT3D(#8,#7,#6);
#10=IFCDIRECTION((0.,1.,0.));
#11=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-05,#9,#10);
#12=IFCDIMENSIONALEXPONENTS(0,0,0,0,0,0,0);
#13=IFCSIUNIT(*,.LENGTHUNIT.,.DECI.,.METRE.);
#14=IFCSIUNIT(*,.AREAUNIT.,.DECI.,.SQUARE_METRE.);
#15=IFCSIUNIT(*,.VOLUMEUNIT.,.DECI.,.CUBIC_METRE.);
#16=IFCSIUNIT(*,.PLANEANGLEUNIT.,$,.RADIAN.);
#17=IFCMEASUREWITHUNIT(IFCPLANEANGLEMEASURE(0.017453292519943295),#16);
#18=IFCCONVERSIONBASEDUNIT(#12,.PLANEANGLEUNIT.,'DEGREE',#17);
#19=IFCUNITASSIGNMENT((#13,#14,#15,#18));
#20=IFCPROJECT('%(project_globalid)s',#5,'%(project_name)s',$,$,$,$,(#11),#19);
ENDSEC;
END-ISO-10303-21;
""" % locals()
    return template
# 三引号字符串所见即所得，不需要转义符号


############################################
#  create ifc template
############################################


############################################
#  Qt GUI
############################################

class MyMainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent = None):
        super(MyMainWindow, self).__init__(parent)
        self.setupUi(self)

        # choose input file and output path
        self.pushButton_choose_urdf_file.clicked.connect(self.inputUrdfileName)
        self.pushButton_choose_ifc_path.clicked.connect(self.inputIfcPath)

        # status display
        self.lineEdit_ifc_path.textChanged.connect(self.ifcStatus)
        self.lineEdit_urdf_file_path.textChanged.connect(self.urdfStatus)

        # URDF display
        self.pushButton_display_urdf.clicked.connect(self.urdfDisplay)

        # connect spinbox with slider
        self.spinBox_scale.valueChanged.connect(self.horizontalSlider_scale.setValue)
        self.horizontalSlider_scale.valueChanged.connect(self.spinBox_scale.setValue)

        # converter
        self.pushButton_start_to_convert.clicked.connect(self.coreConverter)

        # save ifc
        self.pushButton_save_ifc.clicked.connect(self.ifcSave)

        # reset gui
        self.pushButton_reset_gui.clicked.connect(self.resetGui)



    def inputUrdfileName(self):
        # Reset GUI
        self.resetGui()

        urdf_filename_list = QFileDialog.getOpenFileName(self, "Choose Urdf File", os.getcwd(), "URDF Files(*.urdf);;All Files (*)", options=QFileDialog.DontUseNativeDialog)
        urdf_filename = urdf_filename_list[0]
        # print(urdf_filename)
            
        self.lineEdit_urdf_file_path.setText(urdf_filename)
        if self.lineEdit_ifc_path.text() == "":
            self.lineEdit_ifc_path.setText(os.path.dirname(urdf_filename))
        log_print("Urdf File Selecte\n")

        self.pushButton_display_urdf.setEnabled(True)
        self.pushButton_choose_ifc_path.setEnabled(True)
        self.pushButton_start_to_convert.setEnabled(True)



    def inputIfcPath(self):
        ifc_save_path = QFileDialog.getExistingDirectory(self, "Choose ifc Saved Path", os.getcwd(), options=QFileDialog.DontUseNativeDialog)
        if ifc_save_path != "":    
            self.outname = ifc_save_path
            self.lineEdit_ifc_path.setText(ifc_save_path)
            log_print("Path for Saving ifc Selected\n")


    def ifcStatus(self):
        self.label_ifc_process.setText("input")

    def urdfStatus(self):
                 
        validUrdfPath = None  #Target path of urdf file  
        validUrdfPath = checkUrdfPath(self.lineEdit_urdf_file_path.text())
        if validUrdfPath == None:
            # sys.exit('URDF File unvalid!')  # path is unvalid, then exit
            self.label_urdf_valid.setText("Urdf Unvalid")
        else:
            self.label_urdf_valid.setText("Urdf Valid")


    def coreConverter(self):
        
        global scale
        scale = self.spinBox_scale.value()
        log_print('global scale = '+ str(scale))

        #self.timer_init()
        myWin.label_time.setText("Wait...")
        start = time.time()
        validUrdfPath =  self.lineEdit_urdf_file_path.text() # here is text() but not text
        savedPath = self.lineEdit_ifc_path.text()
        urdf2IfcConverter(validUrdfPath, savedPath)
        end = time.time()

        log_print('used time: ' + str(end-start))
        log_print("Converter completed\n")
        
        # set used time on label
        during = int(end-start)
        self.timerString = str(during//60).zfill(2)+":"+str(during%60).zfill(2)
        myWin.label_time.setText(self.timerString)

        #self.timer_stop()

        # 多线程方法
        # self.thread_1 = PreventFastClickThreadMutex()  # 创建线程
        # self.thread_1.start()  # 开始线程

        self.pushButton_save_ifc.setEnabled(True)



    def urdfDisplay(self):
        log_print("Urdf display")
        os.system("urdf-viz"+" "+self.lineEdit_urdf_file_path.text())

    def ifcSave(self):
        log_print("Ifc saved")

        
    def progressBarSet(self):
        global progressBarTotal
        global progressBarCount
        progressBarCount += 1
        self.progressBar_converting.setRange(0, progressBarTotal)
        self.progressBar_converting.setValue(progressBarCount)

    def resetGui(self):
        global progressBarTotal
        global progressBarCount

        progressBarTotal = 0
        progressBarCount = 0

        self.label_urdf_valid.setText("not input")
        self.label_ifc_process.setText("not input")
        self.lineEdit_urdf_file_path.clear()
        self.lineEdit_ifc_path.clear()
        self.progressBar_converting.setValue(0) 
        self.label_time.setText("00:00")
        self.textEdit_terminal.clear()

        self.pushButton_choose_ifc_path.setEnabled(False)
        self.pushButton_start_to_convert.setEnabled(False)
        self.pushButton_display_urdf.setEnabled(False)
        self.pushButton_save_ifc.setEnabled(False)

        self.horizontalSlider_scale.setValue(10)
        self.spinBox_scale.setValue(10)
        


# timer
    def timer_init(self):
        self.timerSec = 0
        self.timer = QTimer(self) #初始化一个定时器
        self.timer.timeout.connect(self.operate) #计时结束调用operate()方法
        self.timer.start(1000) #设置计时间隔并启动,1秒

    def timer_stop(self):
        self.timer.stop()

    def operate(self):
        self.timerSec += 1
        if self.timerSec >= 3600:
            self.timerSec = 0

        self.timerString = str(self.timerSec//60).zfill(2)+":"+str(self.timerSec%60).zfill(2)
        myWin.label_time.setText(self.timerString)
        print("good")

    
############################################
#  Qt GUI
############################################




# class PreventFastClickThreadMutex(QThread):  # 线程1
#     qmut = QMutex()  # 创建线程锁
#     def __init__(self):
#         super().__init__()

#     def run(self):
#         self.qmut.lock()  # 加锁

#         ### Function
#         start = time.time()
#         self.timer_init()
#         validUrdfPath =  myWin.lineEdit_urdf_file_path.text() # here is text() but not text
#         savedPath = myWin.lineEdit_ifc_path.text()

#         urdf2IfcConverter(validUrdfPath, savedPath)
#         end = time.time()

#         log_print('used time: ' + str(end-start))
#         log_print("Converted finished\n")

#         self.timer_stop()
#         ### Function


#         self.qmut.unlock()  # 解锁


#     def timer_init(self):
#         self.timerSec = 0
#         self.timer = QTimer(self) #初始化一个定时器
#         self.timer.timeout.connect(self.operate) #计时结束调用operate()方法
#         self.timer.start(1000) #设置计时间隔并启动,1秒

#     def timer_stop(self):
#         self.timer.stop()

#     def operate(self):
#         self.timerSec += 1
#         if self.timerSec >= 3600:
#             self.timerSec = 0

#         self.timerString = str(self.timerSec//60).zfill(2)+":"+str(self.timerSec%60).zfill(2)
#         myWin.label_time.setText(self.timerString)
#         print("good")





############################################
#  Main
############################################

# # 单独运行qt gui时

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWin = MyMainWindow()

    # Glocal parameters    
    scale = 1 # default

    progressBarTotal = 0
    progressBarCount = 0

    sss =""
    myWin.resetGui()
    myWin.show()

    sys.exit(app.exec_())

############################################
#  Main
############################################




