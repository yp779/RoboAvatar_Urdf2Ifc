#!/usr/bin/env python

"""
Testing the APIs of IfcOpenShell.
"""


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


# get dir1 dir2 for ifclocalplacement from rpy
def rpy_get_dir(therpy):
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

############################################
# tool functions
############################################



############################################
# ifc creation related fuctions
############################################

# 放置位置
def create_ifclocalplacement(ifcfile, point=O, dir1=Z, dir2=X, relative_to=None): # Z,X
    axis2placement = create_ifcaxis2placement(ifcfile, point, dir1, dir2)
    ifclocalplacement2 = ifcfile.createIfcLocalPlacement(relative_to, axis2placement)
    return ifclocalplacement2

# 2D位置
def create_ifcaxis2placement(ifcfile, point=(.0 , .0, .0), dir1=Z, dir2=X):
    point = ifcfile.createIfcCartesianPoint(point)
    dir1 = ifcfile.createIfcDirection(dir1)
    dir2 = ifcfile.createIfcDirection(dir2)
    axis2placement = ifcfile.createIfcAxis2Placement3D(point, dir1, dir2)
    return axis2placement

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

# Create properties function(NEW)
'''def create_properties(ifcfile, propType, )
    ifcfile.'''

############################################
# ifc creation related fuctions
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
    
    #*********START Create Robot Assembly***********#
    #***********************************************#
    rob_nm = "TurtleBot1"    
    robot_assembly = theifcfile.createIfcElementAssembly(GlobalId=create_guid(),
                                                         OwnerHistory=owner_history,
                                                         Name=rob_nm,
                                                         Description="This is TurtleBOt1",
                                                         PredefinedType="USERDEFINED",
                                                         AssemblyPlace="FACTORY")    
    #*********END Create Robot Assembly*************#
    #***********************************************#
    
    #*********START Create Robot Base Links**************#
    #****************************************************#
    link1_nm = "base_link"
    link1_placement = create_ifclocalplacement(theifcfile, relative_to = storey_placement)
    link1Rep_solid = create_ifcextrudedareasolid(theifcfile,                                        
                                        [(-0.25, -0.25, -0.1), (-0.25, 0.25, -0.1), (0.25, 0.25, -0.1), (0.25, -0.25, -0.1)],
                                        create_ifcaxis2placement(theifcfile),
                                        (0.0, 0.0, 1.0),
                                        float(0.2))
    link1Rep_ShapeRep = theifcfile.createIfcShapeRepresentation(context, "robotBody", "Box", [link1Rep_solid])
    link1Rep_Shape = theifcfile.createIfcProductDefinitionShape(None, None, [link1Rep_ShapeRep])                                    
    link1_representation = link1Rep_Shape       
    robot_link1 = theifcfile.createIfcBuildingElementProxy(GlobalId=create_guid(),
                                                           OwnerHistory=owner_history,
                                                           Name=link1_nm,
                                                           ObjectPlacement=link1_placement,
                                                           Representation=link1_representation
                                                           )    
    theifcfile.createIfcRelAggregates(GlobalId=create_guid(),
                                   OwnerHistory=owner_history,
                                   RelatingObject=robot_assembly,
                                   RelatedObjects=[robot_link1])
    colour(theifcfile, [robot_link1], [1.0, 0.0, 0.0])    						                      
    #*********END Create Robot Base Links****************#
    #****************************************************#
    
    #*********START Create Robot Joints**************#
    #***********************************************#
    joint1_nm = "wheel_joint1"
    joint1_decp = "This is the joint connecting the base with wheel1"
    joint1_placement = create_ifclocalplacement(theifcfile, point=[-0.25,0.0,0.0], dir1=(-1., 0., 0.), dir2=(0., 0., 1.),  relative_to = storey_placement)
    wheel_joint1 = theifcfile.createIfcVirtualElement(GlobalId=create_guid(),
                                                      OwnerHistory=owner_history,
                                                      Name=joint1_nm,
                                                      Description=joint1_decp,
                                                      ObjectPlacement=joint1_placement)    
    theifcfile.createIfcRelAggregates(GlobalId=create_guid(),
                                   OwnerHistory=owner_history,
                                   RelatingObject=robot_assembly,
                                   RelatedObjects=[wheel_joint1])
                                   
    joint2_nm = "wheel_joint2"
    joint2_decp = "This is the joint connecting the base with wheel2"
    joint2_placement = create_ifclocalplacement(theifcfile, point=[0.25,0.0,0.0], dir1=(1., 0., 0.), dir2=(0., 1., 0.), relative_to = storey_placement)
    wheel_joint2 = theifcfile.createIfcVirtualElement(GlobalId=create_guid(),
                                                      OwnerHistory=owner_history,
                                                      Name=joint2_nm,
                                                      Description=joint2_decp,
                                                      ObjectPlacement=joint2_placement)    
    theifcfile.createIfcRelAggregates(GlobalId=create_guid(),
                                   OwnerHistory=owner_history,
                                   RelatingObject=robot_assembly,
                                   RelatedObjects=[wheel_joint2])
    #*********END Create Robot Joints****************#
    #***********************************************#
    
    #*********START Create Robot Wheel Links*************#
    #****************************************************#
    wheel_link1_nm = "wheel_link1"
    wheel_link1_decp = "This is the link representing wheel1"
    wheel_link1_placement = create_ifclocalplacement(theifcfile, point=[0.0,0.0,0.0], relative_to = joint1_placement)
    wheel_link1Rep_solid = create_ifcextrudedareasolid_circle(theifcfile,
	                                                          0.15,
	                                                          create_ifcaxis2placement(theifcfile),
	                                                          (0.0, 0.0, 1.0),
	                                                          float(0.2))    
    wheel_link1Rep_ShapeRep = theifcfile.createIfcShapeRepresentation(context, "robotBody", "Cylinder", [wheel_link1Rep_solid])
    wheel_link1Rep_Shape = theifcfile.createIfcProductDefinitionShape(None, None, [wheel_link1Rep_ShapeRep])                                    
    wheel_link1_representation = wheel_link1Rep_Shape       
    robot_wheel_link1 = theifcfile.createIfcBuildingElementProxy(GlobalId=create_guid(),
                                                           OwnerHistory=owner_history,
                                                           Name=wheel_link1_nm,
                                                           Description=wheel_link1_decp,
                                                           ObjectPlacement=wheel_link1_placement,
                                                           Representation=wheel_link1_representation)    
    theifcfile.createIfcRelAggregates(GlobalId=create_guid(),
                                   OwnerHistory=owner_history,
                                   RelatingObject=robot_assembly,
                                   RelatedObjects=[robot_wheel_link1])
    colour(theifcfile, [robot_wheel_link1], [1.0, 0.0, 0.0])          
    
    wheel_link2_nm = "wheel_link2"
    wheel_link2_decp = "This is the link representing wheel2"
    wheel_link2_placement = create_ifclocalplacement(theifcfile, point=[0.0,0.0,0.0], relative_to = joint2_placement)
    wheel_link2Rep_solid = create_ifcextrudedareasolid_circle(theifcfile,
	                                                          0.15,
	                                                          create_ifcaxis2placement(theifcfile),
	                                                          (0.0, 0.0, 1.0),
	                                                          float(0.2))    
    wheel_link2Rep_ShapeRep = theifcfile.createIfcShapeRepresentation(context, "robotBody", "Cylinder", [wheel_link2Rep_solid])
    wheel_link2Rep_Shape = theifcfile.createIfcProductDefinitionShape(None, None, [wheel_link2Rep_ShapeRep])                                    
    wheel_link2_representation = wheel_link2Rep_Shape       
    robot_wheel_link2 = theifcfile.createIfcBuildingElementProxy(GlobalId=create_guid(),
                                                           OwnerHistory=owner_history,
                                                           Name=wheel_link2_nm,
                                                           Description=wheel_link2_decp,
                                                           ObjectPlacement=wheel_link2_placement,
                                                           Representation=wheel_link2_representation)    
    theifcfile.createIfcRelAggregates(GlobalId=create_guid(),
                                   OwnerHistory=owner_history,
                                   RelatingObject=robot_assembly,
                                   RelatedObjects=[robot_wheel_link2])
    colour(theifcfile, [robot_wheel_link2], [1.0, 0.0, 0.0])      
    #*********END Create Robot Wheel Links***************#
    #****************************************************#
    
    #*********START Connect Joints to Links**************#
    #****************************************************#
    theifcfile.createIfcRelAssignsToProduct(GlobalId=create_guid(),
                                      OwnerHistory=owner_history,
                                      Name="parent_base_link_wheel_joint1",
                                      Description="This is relationship assigning wheel_joint1 to its parent base_link",
                                      RelatingProduct=robot_link1,
                                      RelatedObjects=[wheel_joint1])
    theifcfile.createIfcRelAssignsToProduct(GlobalId=create_guid(),
                                      OwnerHistory=owner_history,
                                      Name="child_wheel_joint1_wheel_link1",
                                      Description="This is relationship assigning wheel_link1 to its parent wheel_joint1",
                                      RelatingProduct=wheel_joint1,
                                      RelatedObjects=[robot_wheel_link1])
    theifcfile.createIfcRelAssignsToProduct(GlobalId=create_guid(),
                                      OwnerHistory=owner_history,
                                      Name="parent_base_link_wheel_joint2",
                                      Description="This is relationship assigning wheel_joint2 to its parent base_link",
                                      RelatingProduct=robot_link1,
                                      RelatedObjects=[wheel_joint2])
    theifcfile.createIfcRelAssignsToProduct(GlobalId=create_guid(),
                                      OwnerHistory=owner_history,
                                      Name="child_wheel_joint2_wheel_link2",
                                      Description="This is relationship assigning wheel_link2 to its parent wheel_joint2",
                                      RelatingProduct=wheel_joint2,
                                      RelatedObjects=[robot_wheel_link2])
    #*********END Connect Joints to Links****************#
    #****************************************************#
    
    #*********START Define Type Robot**************#
    #**********************************************#
    robot_type = theifcfile.createIfcTypeObject(GlobalId=create_guid(),
                                                OwnerHistory=owner_history,
                                                Name="RobotType",
                                                Description="This is the type object of robot occurrence",
                                                ApplicableOccurrence="IfcElementAssembly/ROBOT")
    link_type = theifcfile.createIfcTypeObject(GlobalId=create_guid(),
                                               OwnerHistory=owner_history,
                                               Name="LinkType",
                                               Description="This is the type object of robot link occurrence",
                                               ApplicableOccurrence="IfcBuildingElementProxy/LINK")
    joint_type = theifcfile.createIfcTypeObject(GlobalId=create_guid(),
                                                OwnerHistory=owner_history,
                                                Name="JointType",
                                                Description="This is the type object of robot joint occurrence",
                                                ApplicableOccurrence="IfcVirtualElement/JOINT")
    theifcfile.createIfcRelDefinesByType(GlobalId=create_guid(),
                                         OwnerHistory=owner_history,                         
                                         RelatingType=robot_type,
                                         RelatedObjects=[robot_assembly])
    theifcfile.createIfcRelDefinesByType(GlobalId=create_guid(),
                                         OwnerHistory=owner_history,                         
                                         RelatingType=link_type,
                                         RelatedObjects=[robot_link1, robot_wheel_link1, robot_wheel_link2])
    theifcfile.createIfcRelDefinesByType(GlobalId=create_guid(),
                                         OwnerHistory=owner_history,                         
                                         RelatingType=joint_type,
                                         RelatedObjects=[wheel_joint1, wheel_joint2]) 
                                         
    #*********END Define Type Robot****************#
    #**********************************************#
    
    #*********START Define Robot Properties**************#
    #****************************************************#
    # Robot properties
    prop_single_nm_list = ["ThroughPut", "SuccessRate", "NaviSpeed", "BatteryLimit", "Mass", "NumOfParts",
                                 "Sensing", "Grasping", "Climbing", "PickAndPlace"]
    prop_single_val_type_list = ["IfcText", "IfcText", "IfcText", "IfcText", "IfcText", "IfcText",
                                 "IfcText", "IfcText", "IfcText", "IfcText"]
    '''prop_single_val_type_list = ["IfcText", "IfcRatioMeasure", "IfcText", "IfcDuration", "IfcMassMeasure", "IfcInteger",
                                 "IfcBoolean", "IfcBoolean", "IfcBoolean", "IfcBoolean"]'''
    prop_single_val_list = ["15 m3/s", "92%", "0.5 m/s", "P0Y0M3DT0H0M0S", "50 kg", "3", "FLASE", "FLASE", "TRUE", "FALSE"]    
    ifc_prop_val_pairs = []
    for i in range(len(prop_single_nm_list)):
        prop_single_nm = prop_single_nm_list[i]
        prop_single_val_type = prop_single_val_type_list[i]
        prop_single_val = prop_single_val_list[i]
        ifc_prop_val_pairs.append(theifcfile.createIfcPropertySingleValue(prop_single_nm,None,theifcfile.create_entity(prop_single_val_type, prop_single_val),None))
    RobotLocomotionEnumVal = [theifcfile.create_entity("IfcLabel","Wheel"),
                              theifcfile.create_entity("IfcLabel","Leg"),
                              theifcfile.create_entity("IfcLabel","WheelLeg"),
                              theifcfile.create_entity("IfcLabel","Fly"),
                              theifcfile.create_entity("IfcLabel","Fixed")]
    RobotLocomotionEnum = theifcfile.createIfcPropertyEnumeration("RobotLocomotionEnum", RobotLocomotionEnumVal, None)
    ifc_prop_val_pairs.append(theifcfile.createIfcPropertyEnumeratedValue("Locomotion",None,[theifcfile.create_entity("IfcLabel", "Wheel")],RobotLocomotionEnum))    
    ifc_prop_set = theifcfile.createIfcPropertySet(GlobalId=create_guid(),
                                                   OwnerHistory=owner_history,
                                                   Name="RobotPropSet",
                                                   HasProperties=ifc_prop_val_pairs)
    theifcfile.createIfcRelDefinesByProperties(GlobalId=create_guid(),
                                               OwnerHistory=owner_history,                         
                                               RelatingPropertyDefinition=ifc_prop_set,
                                               RelatedObjects=[robot_assembly]) 
   
    # Link properties
    baselink_prop_single_nm_list = ["Mass","Sensing","Grasping","Climbing","PickAndPlace"]
    baselink_prop_single_val_type_list = ["IfcText", "IfcText", "IfcText", "IfcText", "IfcText"]
    baselink_prop_single_val_list = ["35 kg", "FLASE", "FALSE", "FALSE", "FALSE"]
    ifc_baselink_prop_val_pairs = []
    for i in range(len(baselink_prop_single_nm_list)):
        prop_single_nm = baselink_prop_single_nm_list[i]
        prop_single_val_type = baselink_prop_single_val_type_list[i]
        prop_single_val = baselink_prop_single_val_list[i]
        ifc_baselink_prop_val_pairs.append(theifcfile.createIfcPropertySingleValue(prop_single_nm,None,theifcfile.create_entity(prop_single_val_type, prop_single_val),None))        
    InetiaMatValList = [theifcfile.create_entity("IfcText","2.2124416e-03"),
                        theifcfile.create_entity("IfcText","-1.2294101e-05"),
                        theifcfile.create_entity("IfcText","3.4938785e-05"),
                        theifcfile.create_entity("IfcText","2.1193702e-03"),
                        theifcfile.create_entity("IfcText","-5.0120904e-06"),
                        theifcfile.create_entity("IfcText","2.0064271e-03")]
    ifc_baselink_prop_val_pairs.append(theifcfile.createIfcPropertyListValue("InertiaMat","ixx, ixy, ixz, iyy, iyz, izz",InetiaMatValList,None))
    ifc_baselink_prop_set = theifcfile.createIfcPropertySet(GlobalId=create_guid(),
                                                            OwnerHistory=owner_history,
                                                            Name="BaseLinkPropSet",
                                                            HasProperties=ifc_baselink_prop_val_pairs)
    theifcfile.createIfcRelDefinesByProperties(GlobalId=create_guid(),
                                               OwnerHistory=owner_history,                         
                                               RelatingPropertyDefinition=ifc_baselink_prop_set,
                                               RelatedObjects=[robot_link1]) 
        
    # Joint properties
    ifc_wheeljoint1_prop_val_pairs = []
    JointTypeEnumVal = [theifcfile.create_entity("IfcLabel","Revolute"),
                        theifcfile.create_entity("IfcLabel","Continuous"),
                        theifcfile.create_entity("IfcLabel","Prismatic"),
                        theifcfile.create_entity("IfcLabel","Fixed"),
                        theifcfile.create_entity("IfcLabel","Floating"), 
                        theifcfile.create_entity("IfcLabel","Planar")]
    JointTypeEnum = theifcfile.createIfcPropertyEnumeration("JointTypeEnum", JointTypeEnumVal, None)
    ifc_wheeljoint1_prop_val_pairs.append(theifcfile.createIfcPropertyEnumeratedValue("JointMovType",None,[theifcfile.create_entity("IfcLabel", "Continuous")],JointTypeEnum))
    JointMovAxisValList = [theifcfile.create_entity("IfcText","0.0"),
                           theifcfile.create_entity("IfcText","0.0"),
                           theifcfile.create_entity("IfcText","1.0")]
    ifc_wheeljoint1_prop_val_pairs.append(theifcfile.createIfcPropertyListValue("JointMovAxis", "normalized axix vector [x, y, z]", JointMovAxisValList, None))
    JointMovLimitValList = [theifcfile.create_entity("IfcText","None"),
                            theifcfile.create_entity("IfcText","None")]
    ifc_wheeljoint1_prop_val_pairs.append(theifcfile.createIfcPropertyListValue("JointMovLimit", "[lowerLimit, upperlimit] only applied to type Revolute and Prismatic", JointMovLimitValList, None))
    ifc_wheeljoint1_prop_set = theifcfile.createIfcPropertySet(GlobalId=create_guid(),
                                                               OwnerHistory=owner_history,
                                                               Name="WheelJoint1PropSet",
                                                               HasProperties=ifc_wheeljoint1_prop_val_pairs)
    theifcfile.createIfcRelDefinesByProperties(GlobalId=create_guid(),
                                               OwnerHistory=owner_history,                         
                                               RelatingPropertyDefinition=ifc_wheeljoint1_prop_set,
                                               RelatedObjects=[wheel_joint1])     
    
    #*********END Define Robot Properties****************#
    #****************************************************#
    
    # Aggregate the robot assembly to building storey
    theifcfile.createIfcRelAggregates(GlobalId=create_guid(),
                                      OwnerHistory=owner_history,
                                      RelatingObject=building_storey,
                                      RelatedObjects=[robot_assembly])
    
    container_storey = theifcfile.createIfcRelAggregates(create_guid(), owner_history, "Building Container", None, building, [building_storey])
    container_site = theifcfile.createIfcRelAggregates(create_guid(), owner_history, "Site Container", None, site, [building])
    container_project = theifcfile.createIfcRelAggregates(create_guid(), owner_history, "Project Container", None, project, [site])
    flow_placement = create_ifclocalplacement(theifcfile, relative_to=storey_placement)

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
#13=IFCSIUNIT(*,.LENGTHUNIT.,.MILLI.,.METRE.);
#14=IFCSIUNIT(*,.AREAUNIT.,.MILLI.,.SQUARE_METRE.);
#15=IFCSIUNIT(*,.VOLUMEUNIT.,.MILLI.,.CUBIC_METRE.);
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
#  Main
############################################


if __name__ == "__main__":    
    outputIfcFile = create_ifc_template()    
    outputIfcFile.write("./demo.ifc")  

############################################
#  Main
############################################




