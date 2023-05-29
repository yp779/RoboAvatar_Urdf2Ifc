#!/usr/bin/env python

"""
URDF files to ifc converter.
"""

import sys

# Check version of Python
if sys.version_info < (3,7):
    sys.exit('This converter requires Python 3.7 or higher.')

import time
import uuid
import argparse
import os
import ifcopenshell

import tempfile
import urdfpy

# OCC
from OCC.Extend.DataExchange import read_stl_file, read_step_file
from ifcopenshell.geom import main as ifctool
# from OCC.Core.TopoDS import TopoDS_Shape
# from OCC.Display.SimpleGui import init_display

# dae 2 stl
import aspose.threed as a3d

# 异常追溯
import faulthandler
faulthandler.enable()

# 更改
# from collections import Mapping
# from collections.abc import Mapping
# 更改
# from fractions import gcd
# from math import gcd


# creat guid
create_guid = lambda: ifcopenshell.guid.compress(uuid.uuid1().hex)


# axis defines
O = 0., 0., 0.
X = 1., 0., 0.
Y = 0., 1., 0.
Z = 0., 0., 1.


# rpy转换坐标系
def rpy_test(rpy):
    theDir1 = 0., 0., 0
    theDir2 = 0., 0., 0
    if rpy[0]!=0 and rpy[1]==0 and rpy[2]==0 :
            theDir1 = Y
            theDir2 = Z
    elif rpy[0]==0 and rpy[1]!=0 and rpy[2]==0 :
            theDir1 = X
            theDir2 = Z
    elif rpy[0]==0 and rpy[1]==0 and rpy[2]!=0 :
            theDir1 = X
            theDir2 = Y
    elif rpy[0]==0 and rpy[1]==0 and rpy[2]==0 :     ##有点奇怪
            theDir1 = Z
            theDir2 = X      
    return theDir1, theDir2     


# 放置位置
def create_ifclocalplacement(ifcfile, point=O, dir1=Z, dir2=X, relative_to=None):
    axis2placement = create_ifcaxis2placement(ifcfile, point, dir1, dir2)
    ifclocalplacement2 = ifcfile.createIfcLocalPlacement(relative_to, axis2placement)
    return ifclocalplacement2

# 2D位置
def create_ifcaxis2placement(ifcfile, point=(.5 , .5, .5), dir1=Z, dir2=X):
    point = ifcfile.createIfcCartesianPoint(point)
    dir1 = ifcfile.createIfcDirection(dir1)
    dir2 = ifcfile.createIfcDirection(dir2)
    axis2placement = ifcfile.createIfcAxis2Placement3D(point, dir1, dir2)
    return axis2placement

# # 1D位置
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


# 转换urdf颜色到ifc颜色
def colorConverter(link):
    color = [link.visuals[0].material.color[0], link.visuals[0].material.color[1], link.visuals[0].material.color[2]]
    return color


# Check Urdf path exists with </robot> tag
def checkUrdfPath(inputPath=None):
    validPath = None  
    if not inputPath:
        print('''"--input" not specified, a URDF content will be read with "</robot>" tag to stop the reading.''')
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


# the converter
def urdf2IfcConverter(input=None):
    validUrdfPath = None  #Target path of urdf file  
    validUrdfPath = checkUrdfPath(input)
    if validUrdfPath == None:
        sys.exit('URDF File unvalid!')  # path is unvalid, then exit
    else:
        ifcfile = creatIfcTemplate() #create a ifc template

    FileName = os.path.basename(validUrdfPath)[:-5] #delete '.urdf' then output as robotName
    Filedir = os.path.dirname(validUrdfPath)
    outputIfcFile = os.path.join(Filedir, FileName + '.ifc')
    print('Convert ' + os.path.basename(validUrdfPath) + ' to ' + os.path.basename(outputIfcFile))

    ifcfile = addLink2IfcTemplate(ifcfile, validUrdfPath) # add entities
    ifcfile.write(outputIfcFile)    # write file


# load Urdf file and add entities to ifc file
def addLink2IfcTemplate(ifcfile, validUrdfPath):

    robot = urdfpy.URDF.load(validUrdfPath)
    for link in robot.links:
        creat_link(ifcfile, link)
    print("Add links finished")   
    ifcopenshell.geom.viewer(ifcfile) 
    return ifcfile

# judge class of link and then creat(box, cylinder, sphere, mesh)
def creat_link(ifcfile, link):
    try:
        if link.visuals[0].geometry.box != None:
            create_box(ifcfile, link)
            print('Box Built: %s' % link.name)
        elif link.visuals[0].geometry.cylinder != None:
            create_cylinder(ifcfile, link)
            print('Cylinder Built: %s' % link.name)
        elif link.visuals[0].geometry.sphere != None:
            create_sphere(ifcfile, link)
            print('Sphere Built: %s' % link.name)
        elif link.visuals[0].geometry.mesh != None:        
            create_mesh(ifcfile, link)
            print('Mesh Built: %s' % link.name)
        else:
            pass
    except (IndexError):
        print("index out of range")
    except :
        print("Other error")        

# Box
def create_box(ifcfile, link):

    global scale
    global type

    length = float(scale*link.visuals[0].geometry.box.size[0])
    width = float(scale*link.visuals[0].geometry.box.size[1])
    height = float(scale*link.visuals[0].geometry.box.size[2])

    b_color = colorConverter(link)
    #print(b_color)

    b_storey = ifcfile.by_type('IfcBuildingStorey')[0]
    b_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    b_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    b_xyz_rpy = urdfpy.matrix_to_xyz_rpy(link.visuals[0].origin)
    b_xyz = [scale*float(i) for i in b_xyz_rpy[0:3]] #float
    b_rpy = b_xyz_rpy[3:6]

    b_dir1, b_dir2 = rpy_test(b_rpy)
    # print(b_xyz_rpy)
    # print(b_xyz)
    # print(b_rpy)
    # print(b_dir1, b_dir2)

    b_space_placement = create_ifclocalplacement(ifcfile, point=(b_xyz[0], b_xyz[1], b_xyz[2]), dir1 = b_dir1, dir2 = b_dir2)
    b_extrusion_placement = create_ifcaxis2placement(ifcfile)

    b_solid = create_ifcextrudedareasolid(ifcfile,
                                        #[(0.0, 0.0, 0.0), (length, 0.0, 0.0), (length, width, 0.0), (0.0, width, 0.0)],
                                        [(-length/2.0, -width/2.0, -height/2.0), (-length/2.0, width/2.0, -height/2.0), (length/2.0, width/2.0, -height/2.0), (length/2.0, -width/2.0, -height/2.0)],
                                        b_extrusion_placement,
                                        (0.0, 0.0, 1.0),
                                        float(height))

    b_body_representation = ifcfile.createIfcShapeRepresentation(b_context, "robotBody", "Box", [b_solid])
    b_entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [b_body_representation])
    b_entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
    b_entity.OwnerHistory = b_OwnerHistory
    b_entity.Name = link.name
    b_entity.Representation = b_entity_shape
    b_entity.ObjectPlacement = b_space_placement
    b_relatingObject = b_storey
    b_related_objects = []
    b_related_objects.append(b_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                b_OwnerHistory,
                                None,
                                None,
                                b_relatingObject,
                                b_related_objects)
    colour(ifcfile, b_related_objects, b_color)  


# Cylinder
def create_cylinder(ifcfile, link):

    global scale
    global type

    c_scale = scale
    length = float(c_scale*link.visuals[0].geometry.cylinder.length)
    radius = float(c_scale*link.visuals[0].geometry.cylinder.radius)
    
    c_color = colorConverter(link)
    #print(c_color)

    c_storey = ifcfile.by_type('IfcBuildingStorey')[0]
    c_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    c_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    c_xyz_rpy = urdfpy.matrix_to_xyz_rpy(link.visuals[0].origin)
    c_xyz = [c_scale*float(i) for i in c_xyz_rpy[0:3]] #float
    c_rpy = c_xyz_rpy[3:6]

    c_dir1, c_dir2 = rpy_test(c_rpy)
    # print(c_xyz_rpy)
    # print(c_xyz)
    # print(c_rpy)
    # print(c_dir1, c_dir2)

    c_space_placement = create_ifclocalplacement(ifcfile, point=(c_xyz[0], c_xyz[1], c_xyz[2]), dir1 = c_dir1, dir2 = c_dir2)
    c_extrusion_placement = create_ifcaxis2placement(ifcfile, point=(float(0) ,float(0),float(-length/4.0)))

    c_solid = create_ifcextrudedareasolid_circle(ifcfile,
                                                radius,
                                                c_extrusion_placement,
                                                (0.0, 0.0, 1.0),
                                                float(length))

    c_body_representation = ifcfile.createIfcShapeRepresentation(c_context, "robotBody", "Cylinder", [c_solid])
    c_entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [c_body_representation])
    c_entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
    c_entity.OwnerHistory = c_OwnerHistory
    c_entity.Name = link.name
    c_entity.Representation = c_entity_shape
    c_entity.ObjectPlacement = c_space_placement
    c_relatingObject = c_storey
    c_related_objects = []
    c_related_objects.append(c_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                c_OwnerHistory,
                                None,
                                None,
                                c_relatingObject,
                                c_related_objects)
    colour(ifcfile, c_related_objects, c_color) 


# Sphere
def create_sphere(ifcfile, link):

    global scale
    global type

    s_scale = scale
    radius = float(s_scale*link.visuals[0].geometry.sphere.radius)
    
    s_color = colorConverter(link)
    #print(s_color)

    s_storey = ifcfile.by_type('IfcBuildingStorey')[0]
    s_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    s_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    s_xyz_rpy = urdfpy.matrix_to_xyz_rpy(link.visuals[0].origin)
    s_xyz = [s_scale*float(i) for i in s_xyz_rpy[0:3]] #float
    s_rpy = s_xyz_rpy[3:6]

    s_dir1, s_dir2 = rpy_test(s_rpy)
    # print(s_xyz_rpy)
    # print(s_xyz)
    # print(s_rpy)
    # print(s_dir1, s_dir2)

    s_space_placement = create_ifclocalplacement(ifcfile, point=(s_xyz[0], s_xyz[1], s_xyz[2]), dir1 = s_dir1, dir2 = s_dir2)
    s_sphere_placement = create_ifcaxis2placement(ifcfile)

    s_solid = create_ifcextrudedareasolid_ball(ifcfile,
                                                radius,
                                                s_sphere_placement)

    s_body_representation = ifcfile.createIfcShapeRepresentation(s_context, "robotBody", "Sphere", [s_solid])
    s_entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [s_body_representation])
    s_entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
    s_entity.OwnerHistory = s_OwnerHistory
    s_entity.Name = link.name
    s_entity.Representation = s_entity_shape
    s_entity.ObjectPlacement = s_space_placement
    s_relatingObject = s_storey
    s_related_objects = []
    s_related_objects.append(s_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                s_OwnerHistory,
                                None,
                                None,
                                s_relatingObject,
                                s_related_objects)
    colour(ifcfile, s_related_objects, s_color) 


# mesh
def create_mesh(ifcfile, link):
    
    global scale
    global type

    mesh_path = link.visuals[0].geometry.mesh.filename

    # 默认mesh的scale都相等，若不相等报错
    if link.visuals[0].geometry.mesh.scale[0] == link.visuals[0].geometry.mesh.scale[1] == link.visuals[0].geometry.mesh.scale[2]:
        m_scale = float(scale * link.visuals[0].geometry.mesh.scale[0])
    else:
        m_scale = float(1.0)
        print("The mesh scale is error, set it as 1.0")

    m_color = colorConverter(link)
    # print(m_color)

    m_storey = ifcfile.by_type('IfcBuildingStorey')[0]
    m_OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
    m_context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]

    m_xyz_rpy = urdfpy.matrix_to_xyz_rpy(link.visuals[0].origin)
    m_xyz = [m_scale*float(i) for i in m_xyz_rpy[0:3]] #float
    m_rpy = m_xyz_rpy[3:6]

    m_dir1, m_dir2 = rpy_test(m_rpy)
    # print(m_xyz_rpy)
    # print(m_xyz)
    # print(m_rpy)
    # print(m_dir1, m_dir2)

    m_space_placement = create_ifclocalplacement(ifcfile, point=(m_xyz[0], m_xyz[1], m_xyz[2]), dir1 = m_dir1, dir2 = m_dir2)
    
    # Get mesh from .stl or .dae
    m_stlShape = mesh_converter(mesh_path)
    # m_stlShape = read_stl_file("/home/ilab/URDF2IFC/Project/urdf_models/123.stl")
    
    m_entity_shape = ifctool.tesselate(ifcfile.schema, m_stlShape, m_scale * 1.)
    m_entity_shape.Representations[0].ContextOfItems = m_context
    #ifcfile.add(m_entity_shape)

    m_entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
    m_entity.OwnerHistory = m_OwnerHistory
    m_entity.Name = link.name
    m_entity.Representation = m_entity_shape
    m_entity.ObjectPlacement = m_space_placement
    m_relatingObjects = m_storey
    m_related_objects = []
    m_related_objects.append(m_entity)
    ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                   m_OwnerHistory,
                                   None,
                                   None,
                                   m_relatingObjects,
                                   m_related_objects)
    colour(ifcfile, m_related_objects, m_color)


# convert mesh to stl format， support suffix of (.stl .dae .obj)
# .stp/.step is supported by read_step_file method, but not supported by this urdf parser  
def mesh_converter(the_mesh_path):

    mesh_path_suffix = os.path.splitext(os.path.basename(the_mesh_path))[-1]

    if mesh_path_suffix == '.stl':
        m_stlShape = read_stl_file(the_mesh_path)
    elif mesh_path_suffix == '.dae':
        stlOfDae = a3d.Scene.from_file("/home/ilab/URDF2IFC/Project/urdf_models/test.dae")
        path_stlOfDae = os.path.dirname(the_mesh_path)+"tempStlFileForDae.obj"
        stlOfDae.save(path_stlOfDae)
        m_stlShape = read_stl_file(path_stlOfDae)
        os.remove(path_stlOfDae)
    elif mesh_path_suffix == '.obj':
        stlOfObj = a3d.Scene.from_file("/home/ilab/URDF2IFC/Project/urdf_models/airboat.obj")
        path_stlOfObj = os.path.dirname(the_mesh_path)+"tempStlFileForObj.stl"
        stlOfObj.save(path_stlOfObj)
        m_stlShape = read_stl_file(path_stlOfObj)
        os.remove(path_stlOfObj)
    else :
        print('%s not support' % mesh_path_suffix)
        m_stlShape = None

    return m_stlShape


#####


def creatIfcTemplate():
    
    # Write the template to a temporary file
    ifcTemplate = readBasicTemplate()
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

    return theifcfile


def readBasicTemplate():
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
             


if __name__ == '__main__':

    scale = 100
    type = 'IfcBuildingElementProxy'

    # Parsing input path in terminal
    parser = argparse.ArgumentParser(description='usage: %prog --input=my_robot.urdf [options]')
    parser.add_argument('--input', dest='input', default='', help='Specifies the URDF file.')
    args = parser.parse_args()

    # Converter
    start = time.time()
    urdf2IfcConverter(args.input)
    end = time.time()
    print('used time: ', end-start)


    # display, start_display, add_menu, add_function_to_menu = init_display()
    # shp = read_stl_file("/home/ilab/URDF2IFC/turtlebot3proj/turtlebot3/turtlebot3_description/meshes/bases/burger_base.stl")
    # display.DisplayShape(shp, update=True)
    # time.sleep(100000)


    ### xacro 2 urdf
    ### rosrun xacro turtlebot3_burger.urdf.xacro --inorder > 123.urdf
