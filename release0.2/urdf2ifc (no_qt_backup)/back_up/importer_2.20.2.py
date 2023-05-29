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
import errno
import argparse
import os
from xml.dom import minidom

import urdf2ifc.parserURDF

import ifcopenshell
import ifcopenshell.geom
import ifcopenshell.util
from ifcopenshell.util.element import copy_deep

import tempfile

import pickle



'''
# String 大写
def convertLUtoUN(s): 
    r = ''
    i = 0
    while i < len(s):
        if i == 0:
            r += s[i].upper()
            i += 1
        elif s[i] == '_' and i < (len(s) - 1):
            r += s[i + 1].upper()
            i += 2
        else:
            r += s[i]
            i += 1
    return r
'''


create_guid = lambda: ifcopenshell.guid.compress(uuid.uuid1().hex)


#create a tempt direcotry not to cover existing directories
def mkdirSafe(directory):
    """Create a dir safely."""
    try:
        os.makedirs(directory)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
        else:
            print('Directory "' + directory + '" already exists!')


#read out Urdf file if exist and has </robot> tag
def convertUrdfFile(input=None, robotName=None):
    """Convert a URDF file into a Webots PROTO file or Robot node string."""
    urdfContent = None  
    if not input:
        print('''"--input" not specified, a URDF content will be read with "</robot>" tag to stop the reading.''')
        urdfContent = ""            #empty urdfContent
        for line in sys.stdin:
            urdfContent += line
            if "</robot>" == line.strip():  # if this line has only </robot>, then break
                break
        print("URDF lecture is finished!")
    else:
        if not os.path.isfile(input):           #input is not a file
            sys.exit('Input file "%s" does not exists.' % input)
        if not input.endswith('.urdf'):         #input is not a urdf file
            sys.exit('"%s" is not a URDF file.' % input)

        with open(input, 'r') as file:                  #open input file with 'r'(read only)
            urdfContent = file.read()
        if urdfContent is None:
            sys.exit('Could not read the URDF file.')   #could not read

        convertUrdfFile.urdfPath = os.path.abspath(input) #full name of path, abspath = dirname + basename
        #print(convertUrdfFile.urdfPath)

    return convertUrdfContent(urdfContent, robotName) #input urdfContent for parsering


def convertUrdfContent(urdfContent, robotName=None):   #Convert a URDF content string into a ifc file
   
    urdfDirectory = os.path.dirname(convertUrdfFile.urdfPath)   #urdfDictory is the dirname of input Urdf file
    
    # Required resets in case of multiple conversions
    urdf2ifc.parserURDF.Material.namedMaterial.clear()
    urdf2ifc.parserURDF.Geometry.reference.clear()

    # Convert the content into Webots robot
    domFile = minidom.parseString(urdfContent)
    for child in domFile.childNodes:
        if child.localName == 'robot':  # root tag
  
            robotName = os.path.basename(convertUrdfFile.urdfPath)[:-5] #delete '.urdf' then output as robotName
            Filedir = os.path.dirname(convertUrdfFile.urdfPath)
            outputFile = os.path.join(Filedir, robotName + '.ifc')
            print('Convert ' + os.path.basename(convertUrdfFile.urdfPath) + ' to ' + os.path.basename(outputFile))

            mkdirSafe(robotName + '_textures')  # make a dir called 'x_textures'

            urdf2ifc.parserURDF.robotName = robotName  # give robotName to parser

            linkElementList = []
            jointElementList = []

            robot = child
            for child in robot.childNodes:
                if child.localName == 'link':           # link tag
                    linkElementList.append(child)
                elif child.localName == 'joint':        # joint tag
                    jointElementList.append(child)
                elif child.localName == 'material':     # material tag
                    if not child.hasAttribute('name') \
                        or child.getAttribute('name') not in urdf2ifc.parserURDF.Material.namedMaterial:
                        material = urdf2ifc.parserURDF.Material()
                        material.parseFromMaterialNode(child)

            # Lists for Elements in URDF
            linkList = []
            jointList = []
            parentList = []
            childList = []
            rootLink = urdf2ifc.parserURDF.Link()   # rootLink is the Link class in parserURDF

            # Generating links
            for link in linkElementList:
                linkList.append(urdf2ifc.parserURDF.getLink(link, urdfDirectory))       # linkList -> parsing each link tag

            # Generating joints    
            for joint in jointElementList:
                jointList.append(urdf2ifc.parserURDF.getJoint(joint))       # jointList -> parsing each joint tag

            # Generating parent & child for links
            for joint in jointList:
                parentList.append(joint.parent)
                childList.append(joint.child)
            parentList.sort()
            childList.sort()
            for link in linkList:
                if urdf2ifc.parserURDF.isRootLink(link.name, childList):
                    # We want to skip links between the robot and the static environment.
                    rootLink = link
                    previousRootLink = link
                    while rootLink in ['base_link', 'base_footprint']:
                        directJoints = []
                        for joint in jointList:
                            if joint.parent == rootLink.name:
                                directJoints.append(joint)
                        if len(directJoints) == 1:
                            for childLink in linkList:
                                if childLink.name == directJoints[0].child:
                                    previousRootLink = rootLink
                                    rootLink = childLink
                        else:
                            rootLink = previousRootLink
                            break

                    print('Root link: ' + rootLink.name)
                    break

            print('There are %d links, %d joints' % (len(linkList), len(jointList)))
            #print(linkElementList[0].inertia.ixx)

            for link in linkList:
                print(link.visual)

            # Write the template to a temporary file
            ifcTemplate = readTemplate()
            temp_handle, temp_filename = tempfile.mkstemp(suffix=".ifc")
            with open(temp_filename, "wb") as f:
                f.write(ifcTemplate.encode())


            ifcfile = ifcopenshell.open(temp_filename)
            owner_history = ifcfile.by_type("IfcOwnerHistory")[0]
            project = ifcfile.by_type("IfcProject")[0]
            context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]
            site_placement = create_ifclocalplacement(ifcfile)
            site = ifcfile.createIfcSite(create_guid(), owner_history, "Site", None, None, site_placement, None, None, "ELEMENT",
                                        None, None, None, None, None)
            building_placement = create_ifclocalplacement(ifcfile, relative_to=site_placement)
            building = ifcfile.createIfcBuilding(create_guid(), owner_history, 'Building', None, None, building_placement, None,
                                                None, "ELEMENT", None, None, None)
            storey_placement = create_ifclocalplacement(ifcfile, relative_to=building_placement)
            elevation = 0.0
            building_storey = ifcfile.createIfcBuildingStorey(create_guid(), owner_history, 'Storey', None, None,
                                                            storey_placement,
                                                            None, None, "ELEMENT", elevation)
            container_storey = ifcfile.createIfcRelAggregates(create_guid(), owner_history, "Building Container", None,
                                                            building,
                                                            [building_storey])
            container_site = ifcfile.createIfcRelAggregates(create_guid(), owner_history, "Site Container", None, site,
                                                            [building])
            container_project = ifcfile.createIfcRelAggregates(create_guid(), owner_history, "Project Container", None, project,
                                                            [site])
            flow_placement = create_ifclocalplacement(ifcfile, relative_to=storey_placement)




            type = 'IfcAirTerminalBox'

            _bottom_height = 0.0
            _wall_height = 100.0
            # color = [0.0, 0.98, 0.99]
            color = [0.98, 0.0, 0.99]

            storey = ifcfile.by_type('IfcBuildingStorey')[0]
            OwnerHistory = ifcfile.by_type('IfcOwnerHistory')[0]
            context = ifcfile.by_type("IfcGeometricRepresentationContext")[0]
            space_placement = create_ifclocalplacement(ifcfile)
            extrusion_placement = create_ifcaxis2placement(ifcfile, point=(0.0, 0.0, float(_bottom_height)))

            solid = create_ifcextrudedareasolid(ifcfile,
                                                [(0.0, 0.0, 0.0), (200.0, 0.0, 0.0), (200.0, 300.0, 0.0), (0.0, 300.0, 0.0)],
                                                extrusion_placement,
                                                (0.0, 0.0, 1.0),
                                                float(_wall_height))

            body_representation = ifcfile.createIfcShapeRepresentation(context, "Body", "SweptSolid", [solid])
            entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [body_representation])
            entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
            entity.OwnerHistory = OwnerHistory
            entity.Name = '111'
            entity.Representation = entity_shape
            entity.ObjectPlacement = space_placement
            relatingObject = storey
            related_objects = []
            related_objects.append(entity)
            ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                        OwnerHistory,
                                        None,
                                        None,
                                        relatingObject,
                                        related_objects)
            colour(ifcfile, related_objects, color)  

                
                
            _wall_height = 50.0

            z_extrusion_placement = create_ifcaxis2placement(ifcfile, point=(200.0, 200.0, float(_bottom_height)))
            z_solid = create_ifcextrudedareasolid_circle(ifcfile,
                                                        50.0,
                                                        z_extrusion_placement,
                                                        (0.0, 0.0, 1.0),
                                                        float(_wall_height))

            z_body_representation = ifcfile.createIfcShapeRepresentation(context, "2Body", "SweptSolid", [z_solid])
            entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [z_body_representation])
            z_entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
            z_entity.OwnerHistory = OwnerHistory
            z_entity.Name = '222'
            z_entity.Representation = entity_shape
            z_entity.ObjectPlacement = space_placement
            z_relatingObject = storey
            z_related_objects = []
            z_related_objects.append(z_entity)
            ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                        OwnerHistory,
                                        None,
                                        None,
                                        z_relatingObject,
                                        z_related_objects)
            colour(ifcfile, z_related_objects, color) 


            z_extrusion_placement = create_ifcaxis2placement(ifcfile, point=(400.0, 400.0, float(_bottom_height)))
            z_solid = create_ifcextrudedareasolid_circle(ifcfile,
                                                        50.0,
                                                        z_extrusion_placement,
                                                        (0.0, 0.0, 1.0),
                                                        float(_wall_height))

            z_body_representation = ifcfile.createIfcShapeRepresentation(context, "2Body", "SweptSolid", [z_solid])
            entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [z_body_representation])
            z_entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
            z_entity.OwnerHistory = OwnerHistory
            z_entity.Name = '333'
            z_entity.Representation = entity_shape
            z_entity.ObjectPlacement = space_placement
            z_relatingObject = storey
            z_related_objects = []
            z_related_objects.append(z_entity)
            ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                        OwnerHistory,
                                        None,
                                        None,
                                        z_relatingObject,
                                        z_related_objects)
            colour(ifcfile, z_related_objects, color) 


            z_extrusion_placement = create_ifcaxis2placement(ifcfile, point=(200.0, 400.0, float(_bottom_height)))
            z_solid = create_ifcextrudedareasolid_circle(ifcfile,
                                                        50.0,
                                                        z_extrusion_placement,
                                                        (0.0, 0.0, 1.0),
                                                        float(_wall_height))

            z_body_representation = ifcfile.createIfcShapeRepresentation(context, "2Body", "SweptSolid", [z_solid])
            entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [z_body_representation])
            z_entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
            z_entity.OwnerHistory = OwnerHistory
            z_entity.Name = '444'
            z_entity.Representation = entity_shape
            z_entity.ObjectPlacement = space_placement
            z_relatingObject = storey
            z_related_objects = []
            z_related_objects.append(z_entity)
            ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                        OwnerHistory,
                                        None,
                                        None,
                                        z_relatingObject,
                                        z_related_objects)
            colour(ifcfile, z_related_objects, color) 


            z_extrusion_placement = create_ifcaxis2placement(ifcfile, point=(400.0, 200.0, float(_bottom_height)))
            z_solid = create_ifcextrudedareasolid_circle(ifcfile,
                                                        50.0,
                                                        z_extrusion_placement,
                                                        (0.0, 0.0, 1.0),
                                                        float(_wall_height))

            z_body_representation = ifcfile.createIfcShapeRepresentation(context, "2Body", "SweptSolid", [z_solid])
            entity_shape = ifcfile.createIfcProductDefinitionShape(None, None, [z_body_representation])
            z_entity = ifcfile.create_entity(type, ifcopenshell.guid.new())
            z_entity.OwnerHistory = OwnerHistory
            z_entity.Name = '555'
            z_entity.Representation = entity_shape
            z_entity.ObjectPlacement = space_placement
            z_relatingObject = storey
            z_related_objects = []
            z_related_objects.append(z_entity)
            ifcfile.createIfcRelAggregates(ifcopenshell.guid.new(),
                                        OwnerHistory,
                                        None,
                                        None,
                                        z_relatingObject,
                                        z_related_objects)
            colour(ifcfile, z_related_objects, color) 
            

            ifcfile.write(robotName+'_urdf2ifc.ifc')

#####

O = 0., 0., 0.
X = 1., 0., 0.
Y = 0., 1., 0.
Z = 0., 0., 1.

# 放置位置
def create_ifclocalplacement(ifcfile, point=O, dir1=Z, dir2=X, relative_to=None):
    axis2placement = create_ifcaxis2placement(ifcfile, point, dir1, dir2)
    ifclocalplacement2 = ifcfile.createIfcLocalPlacement(relative_to, axis2placement)
    return ifclocalplacement2

# 相对放置位置
def create_ifcaxis2placement(ifcfile, point=O, dir1=Z, dir2=X):
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
    circle = create_ifccircle(ifcfile, ifcaxis2placement, radius)
    ifcclosedprofile = ifcfile.createIfcArbitraryClosedProfileDef("AREA", None, circle)
    ifcdir = ifcfile.createIfcDirection(extrude_dir)
    ifcextrudedareasolid = ifcfile.createIfcExtrudedAreaSolid(ifcclosedprofile, ifcaxis2placement, ifcdir, extrusion)
    return ifcextrudedareasolid  
                

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


#####


def readTemplate():
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
    start = time.time()

    parser = argparse.ArgumentParser(description='usage: %prog --input=my_robot.urdf [options]')
    parser.add_argument('--input', dest='input', default='', help='Specifies the URDF file.')
    parser.add_argument('--robot-name', dest='robotName', default=None, help='Specifies the name of the robot.')

    args = parser.parse_args()
    convertUrdfFile(args.input, args.robotName)

    end = time.time()
    print('used time: ', end-start)