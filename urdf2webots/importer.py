#!/usr/bin/env python

"""URDF files to Webots PROTO or Robot node converter."""


import errno
import optparse
import os
import re
import sys
import tempfile
from xml.dom import minidom

import urdf2webots.parserURDF
import urdf2webots.writeRobot

try:
    import rospkg
except ImportError:
    pass

try:
    from ament_index_python import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    pass


def convertLUtoUN(s):
    """Capitalize a string."""
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


def mkdirSafe(directory):
    """Create a dir safely."""
    try:
        os.makedirs(directory)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
        else:
            print('Directory "' + directory + '" already exists!')


def convertUrdfFile(input = None, output=None, robotName=None, normal=False, boxCollision=False,
                 disableMeshOptimization=False, enableMultiFile=False,
                 toolSlot=None, initTranslation='0 0 0', initRotation='0 0 1 0',
                 initPos=None, linkToDef=False, jointToDef=False):
    """Convert a URDF file into a Webots PROTO file or Robot node string."""
    urdfContent = None
    if not input:
        print('''"--input" not specified, a URDF content will be read in the in stdin.\n
            The "</robot>" tag will stop the reading.''')
        urdfContent = ""
        for line in sys.stdin:
            urdfContent += line
            if "</robot>" == line.strip():
                break
        print("URDF lecture is finished!")
    else:
        if not os.path.isfile(input):
            sys.exit('Input file "%s" does not exists.' % input)
        if not input.endswith('.urdf'):
            sys.exit('"%s" is not a URDF file.' % input)

        with open(input, 'r') as file:
            urdfContent = file.read()
        if urdfContent is None:
            sys.exit('Could not read the URDF file.')

        # Set urdfPath for replacing "package://(.*)" occurences later
        convertUrdfFile.urdfPath = os.path.abspath(input)

    return convertUrdfContent(urdfContent, output, robotName, normal, boxCollision,
                 disableMeshOptimization, enableMultiFile,
                 toolSlot, initTranslation, initRotation,
                 initPos, linkToDef, jointToDef)


convertUrdfFile.urdfPath = None


def convertUrdfContent(input, output=None, robotName=None, normal=False, boxCollision=False,
                 disableMeshOptimization=False, enableMultiFile=False,
                 toolSlot=None, initTranslation='0 0 0', initRotation='0 0 1 0',
                 initPos=None, linkToDef=False, jointToDef=False):
    """
    Convert a URDF content string into a Webots PROTO file or Robot node string.
    The current working directory will be used for relative paths in your URDF file.
    To use the location of your URDF file for relative paths, please use the convertUrdfFile() function.
    """
    # Retrieve urdfPath if this function has been called from convertUrdfFile()
    if convertUrdfFile.urdfPath is not None:
        urdfPath = convertUrdfFile.urdfPath
        convertUrdfFile.urdfPath = None
    else:
        urdfPath = os.getcwd()

    if not type(initTranslation) == str or len(initTranslation.split()) != 3:
        sys.exit('--translation argument is not valid. It has to be of Type = str and contain 3 values.')
    if not type(initRotation) == str or len(initRotation.split()) != 4:
        sys.exit('--rotation argument is not valid. It has to be of Type = str and contain 4 values.')
    if initPos is not None:
        try:
            initPos = initPos.replace(",", ' ').replace("[", '').replace("]", '').replace("(", '').replace(")", '')
            initPos = list(map(float, initPos.split()))
        except Exception as e:
            sys.exit(e, '\n--init-pos argument is not valid. Your list has to be inside of quotation marks. '
                     'Example: --init-pos="1.0, 2, -0.4"')

    if robotName:
        if robotName == '':
            sys.exit('--robot-name argument is not valid. It cannot be an empty string.')
        isProto = False
    else:
        isProto = True

    urdf2webots.writeRobot.isProto = isProto
    urdf2webots.parserURDF.disableMeshOptimization = disableMeshOptimization
    urdf2webots.writeRobot.enableMultiFile = enableMultiFile
    urdf2webots.writeRobot.initPos = initPos
    if isProto:
        urdf2webots.writeRobot.toolSlot = toolSlot
        urdf2webots.writeRobot.linkToDef = linkToDef
        urdf2webots.writeRobot.jointToDef = jointToDef
    else:
        urdf2webots.writeRobot.toolSlot = None
        urdf2webots.writeRobot.linkToDef = False
        urdf2webots.writeRobot.jointToDef = False

    # Required resets in case of multiple conversions
    urdf2webots.writeRobot.indexSolid = 0
    urdf2webots.writeRobot.staticBase = False
    urdf2webots.parserURDF.Material.namedMaterial.clear()
    urdf2webots.parserURDF.Geometry.reference.clear()

    # Replace "package://(.*)" occurences
    for match in re.finditer('"package://(.*)"', input):
        packageName = match.group(1).split('/')[0]
        directory = os.path.dirname(urdfPath)
        while packageName != os.path.split(directory)[1] and os.path.split(directory)[1]:
            directory = os.path.dirname(directory)
        if not os.path.split(directory)[1]:
            if 'ROS_VERSION' in os.environ:
                if os.environ['ROS_VERSION'] == '1':
                    try:
                        rospack = rospkg.RosPack()
                        directory = rospack.get_path(packageName)
                    except rospkg.common.ResourceNotFound:
                        sys.stderr.write('Package "%s" not found.\n' % packageName)
                    except NameError:
                        sys.stderr.write('Impossible to find location of "%s" package, installing "rospkg" might help.\n'
                                        % packageName)
                else:
                    try:
                        directory = get_package_share_directory(packageName)
                    except PackageNotFoundError:
                        sys.stderr.write('Package "%s" not found.\n' % packageName)
            else:
                sys.stderr.write('ROS not sourced, package "%s" will not be found.\n' % packageName)
        if os.path.split(directory)[1]:
            packagePath = os.path.split(directory)[0]
            input = input.replace('package://'+packageName, packagePath+'/'+packageName)
        else:
            sys.stderr.write('Can\'t determine package root path.\n')

    # Convert the content into Webots robot
    domFile = minidom.parseString(input)
    for child in domFile.childNodes:
        if child.localName == 'robot':
            if isProto:
                if output:
                    if os.path.splitext(os.path.basename(output))[1] == '.proto':
                        robotName = os.path.splitext(os.path.basename(output))[0]
                        outputFile = output
                    else:
                        # treat output as directory and construct filename
                        robotName = convertLUtoUN(urdf2webots.parserURDF.getRobotName(child))  # capitalize
                        outputFile = os.path.join(output, robotName + '.proto')
                else:
                    robotName = convertLUtoUN(urdf2webots.parserURDF.getRobotName(child))  # capitalize
                    outputFile = output if output else robotName + '.proto'

                mkdirSafe(outputFile.replace('.proto', '') + '_textures')  # make a dir called 'x_textures'

                if enableMultiFile:
                    mkdirSafe(outputFile.replace('.proto', '') + '_meshes')  # make a dir called 'x_meshes'
                    urdf2webots.writeRobot.meshFilesPath = outputFile.replace('.proto', '') + '_meshes'

                protoFile = open(outputFile, 'w')
                urdf2webots.writeRobot.header(protoFile, urdfPath, robotName)
            else:
                tmp_robot_file = tempfile.NamedTemporaryFile(mode="w+", prefix='tempRobotURDFStringWebots')

            urdf2webots.writeRobot.robotName = robotName
            urdf2webots.parserURDF.robotName = robotName  # pass robotName

            robot = child
            linkElementList = []
            jointElementList = []
            for child in robot.childNodes:
                if child.localName == 'link':
                    linkElementList.append(child)
                elif child.localName == 'joint':
                    jointElementList.append(child)
                elif child.localName == 'material':
                    if not child.hasAttribute('name') \
                        or child.getAttribute('name') not in urdf2webots.parserURDF.Material.namedMaterial:
                        material = urdf2webots.parserURDF.Material()
                        material.parseFromMaterialNode(child)

            linkList = []
            jointList = []
            parentList = []
            childList = []
            rootLink = urdf2webots.parserURDF.Link()

            for link in linkElementList:
                linkList.append(urdf2webots.parserURDF.getLink(link, os.path.dirname(urdfPath)))
            for joint in jointElementList:
                jointList.append(urdf2webots.parserURDF.getJoint(joint))
            if not isProto:
                urdf2webots.parserURDF.removeDummyLinks(linkList, jointList)

            for joint in jointList:
                parentList.append(joint.parent)
                childList.append(joint.child)
            parentList.sort()
            childList.sort()
            for link in linkList:
                if urdf2webots.parserURDF.isRootLink(link.name, childList):
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

            for child in robot.childNodes:
                if child.localName == 'gazebo':
                    urdf2webots.parserURDF.parseGazeboElement(child, rootLink.name, linkList)

            sensorList = (urdf2webots.parserURDF.IMU.list +
                            urdf2webots.parserURDF.P3D.list +
                            urdf2webots.parserURDF.Camera.list +
                            urdf2webots.parserURDF.Lidar.list)
            print('There are %d links, %d joints and %d sensors' % (len(linkList), len(jointList), len(sensorList)))

            if isProto:
                urdf2webots.writeRobot.declaration(protoFile, robotName, initTranslation, initRotation)
                urdf2webots.writeRobot.URDFLink(protoFile, rootLink, 1, parentList, childList, linkList, jointList,
                                                sensorList, boxCollision=boxCollision, normal=normal, robot=True)
                protoFile.write('}\n')
                protoFile.close()
                return
            else:
                urdf2webots.writeRobot.URDFLink(tmp_robot_file, rootLink, 0, parentList,
                            childList, linkList, jointList, sensorList, boxCollision=boxCollision,
                            normal=normal, robot=True, initTranslation=initTranslation, initRotation=initRotation)

                tmp_robot_file.seek(0)
                return (tmp_robot_file.read())
    sys.exit('Could not parse the URDF file.\n')


if __name__ == '__main__':
    optParser = optparse.OptionParser(usage='usage: %prog --input=my_robot.urdf [options]')
    optParser.add_option('--input', dest='input', default='', help='Specifies the URDF file.')
    optParser.add_option('--output', dest='output', default='', help='Specifies the path and, if ending in ".proto", name '
                         'of the resulting PROTO file. The filename minus the .proto extension will be the robot name (for PROTO conversion only).')
    optParser.add_option('--robot-name', dest='robotName', default=None, help='Specifies the name of the robot '
                         'and generate a Robot node string instead of a PROTO file (has to be unique).')
    optParser.add_option('--normal', dest='normal', action='store_true', default=False,
                         help='If set, the normals are exported if present in the URDF definition.')
    optParser.add_option('--box-collision', dest='boxCollision', action='store_true', default=False,
                         help='If set, the bounding objects are approximated using boxes.')
    optParser.add_option('--disable-mesh-optimization', dest='disableMeshOptimization', action='store_true', default=False,
                         help='If set, the duplicated vertices are not removed from the meshes (this can speed up a lot the '
                         'conversion).')
    optParser.add_option('--multi-file', dest='enableMultiFile', action='store_true', default=False,
                         help='If set, the mesh files are exported as separated PROTO files.')
    optParser.add_option('--tool-slot', dest='toolSlot', default=None,
                         help='Specify the link that you want to add a tool slot too (exact link name from URDF, for PROTO conversion only).')
    optParser.add_option('--translation', dest='initTranslation', default='0 0 0',
                         help='Set the translation field of your PROTO file or Webots VRML robot string.')
    optParser.add_option('--rotation', dest='initRotation', default='0 0 1 0',
                         help='Set the rotation field of your PROTO file or Webots Robot node string.')
    optParser.add_option('--init-pos', dest='initPos', default=None,
                         help='Set the initial positions of your robot joints. Example: --init-pos="[1.2, 0.5, -1.5]" would '
                         'set the first 3 joints of your robot to the specified values, and leave the rest with their '
                         'default value.')
    optParser.add_option('--link-to-def', dest='linkToDef', action='store_true', default=False,
                         help='Creates a DEF with the link name for each solid to be able to access it using getFromProtoDef(defName) (for PROTO conversion only).')
    optParser.add_option('--joint-to-def', dest='jointToDef', action='store_true', default=False,
                         help='Creates a DEF with the joint name for each joint to be able to access it using getFromProtoDef(defName) (for PROTO conversion only).')
    options, args = optParser.parse_args()
    convertUrdfFile(options.input, options.output, options.robotName, options.normal, options.boxCollision, options.disableMeshOptimization,
                 options.enableMultiFile, options.toolSlot, options.initTranslation, options.initRotation, options.initPos, options.linkToDef, options.jointToDef)
