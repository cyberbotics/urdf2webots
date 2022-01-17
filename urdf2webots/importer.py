#!/usr/bin/env python

"""URDF files to Webots PROTO or robot converter."""


import os
import errno
import optparse
import tempfile
import re
import sys
import urdf2webots.parserURDF
import urdf2webots.writeRobot
from xml.dom import minidom

try:
    import rospkg
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


def convert2urdf(inFile, outFile=None, isProto=True, robotName='', normal=False, boxCollision=False,
                 disableMeshOptimization=False, enableMultiFile=False,
                 toolSlot=None, initTranslation='0 0 0', initRotation='0 0 1 0',
                 initPos=None, linkToDef=False, jointToDef=False):
    if not inFile:
        sys.exit('Input file not specified (should be specified with the "--input" argument).')
    if not os.path.isfile(inFile):
        sys.exit('Input file "%s" does not exists.' % inFile)
    if not inFile.endswith('.urdf'):
        sys.exit('"%s" is not an URDF file.' % inFile)

    if not type(initTranslation) == str or len(initTranslation.split()) != 3:
        sys.exit('--translation argument is not valid. Has to be of Type = str and contain 3 values.')
    if not type(initRotation) == str or len(initRotation.split()) != 4:
        sys.exit('--rotation argument is not valid. Has to be of Type = str and contain 4 values.')
    if initPos is not None:
        try:
            initPos = initPos.replace(",", ' ').replace("[", '').replace("]", '').replace("(", '').replace(")", '')
            initPos = list(map(float, initPos.split()))
        except Exception as e:
            sys.exit(e, '\n--init-pos argument is not valid. Your list has to be inside of quotation marks. '
                     'Example: --init-pos="1.0, 2, -0.4"')

    urdf2webots.writeRobot.isProto = isProto
    urdf2webots.parserURDF.disableMeshOptimization = disableMeshOptimization
    urdf2webots.writeRobot.enableMultiFile = enableMultiFile
    urdf2webots.writeRobot.toolSlot = toolSlot
    urdf2webots.writeRobot.initPos = initPos
    urdf2webots.writeRobot.linkToDef = linkToDef
    urdf2webots.writeRobot.jointToDef = jointToDef
    urdf2webots.writeRobot.indexSolid = 0
    urdf2webots.writeRobot.staticBase = False
    urdf2webots.parserURDF.Material.namedMaterial.clear()
    urdf2webots.parserURDF.Geometry.reference.clear()

    with open(inFile, 'r') as file:
        inPath = os.path.dirname(os.path.abspath(inFile))
        content = file.read()

        for match in re.finditer('"package://(.*)"', content):
            packageName = match.group(1).split('/')[0]
            directory = inPath
            while packageName != os.path.split(directory)[1] and os.path.split(directory)[1]:
                directory = os.path.dirname(directory)
            if not os.path.split(directory)[1]:
                try:
                    rospack = rospkg.RosPack()
                    directory = rospack.get_path(packageName)
                except rospkg.common.ResourceNotFound:
                    sys.stderr.write('Package "%s" not found.\n' % packageName)
                except NameError:
                    sys.stderr.write('Impossible to find location of "%s" package, installing "rospkg" might help.\n'
                                     % packageName)
            if os.path.split(directory)[1]:
                packagePath = os.path.split(directory)[0]
                content = content.replace('package://'+packageName, packagePath+'/'+packageName)
            else:
                sys.stderr.write('Can\'t determine package root path.\n')

        domFile = minidom.parseString(content)

        for child in domFile.childNodes:
            if child.localName == 'robot':
                if isProto:
                    if outFile:
                        if os.path.splitext(os.path.basename(outFile))[1] == '.proto':
                            robotName = os.path.splitext(os.path.basename(outFile))[0]
                            outputFile = outFile
                        else:
                            # treat outFile as directory and construct filename
                            robotName = convertLUtoUN(urdf2webots.parserURDF.getRobotName(child))  # capitalize
                            outputFile = os.path.join(outFile, robotName + '.proto')
                    else:
                        robotName = convertLUtoUN(urdf2webots.parserURDF.getRobotName(child))  # capitalize
                        outputFile = outFile if outFile else robotName + '.proto'

                    mkdirSafe(outputFile.replace('.proto', '') + '_textures')  # make a dir called 'x_textures'

                    if enableMultiFile:
                        mkdirSafe(outputFile.replace('.proto', '') + '_meshes')  # make a dir called 'x_meshes'
                        urdf2webots.writeRobot.meshFilesPath = outputFile.replace('.proto', '') + '_meshes'

                    protoFile = open(outputFile, 'w')
                    urdf2webots.writeRobot.header(protoFile, inFile, robotName)
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
                    linkList.append(urdf2webots.parserURDF.getLink(link, inPath))
                for joint in jointElementList:
                    jointList.append(urdf2webots.parserURDF.getJoint(joint))
                if not isProto:
                    urdf2webots.parserURDF.cleanDummyLinks(linkList, jointList)

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
    print('Could not read file')


if __name__ == '__main__':
    optParser = optparse.OptionParser(usage='usage: %prog --input=my_robot.urdf [options]')
    optParser.add_option('--input', dest='inFile', default='', help='Specifies the urdf file to convert.')
    optParser.add_option('--output', dest='outFile', default='', help='Specifies the path and, if ending in ".proto", name '
                         'of the resulting PROTO file. The filename minus the .proto extension will be the robot name (for PROTO conversion only).')
    optParser.add_option('--is-proto', dest='isProto', default=True, help='Specifies if the conversion tool will create a PROTO file or '
                         'output a Webots VRML robot string.')
    optParser.add_option('--robotName', dest='robotName', default='', help='Specifies the name of the robot (has to be specified if --is-proto is set to False).')
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
                         help='Specify the link that you want to add a tool slot too (exact link name from urdf, for PROTO conversion only).')
    optParser.add_option('--translation', dest='initTranslation', default='0 0 0',
                         help='Set the translation field of your PROTO file or Webots VRML robot string.')
    optParser.add_option('--rotation', dest='initRotation', default='0 0 1 0',
                         help='Set the rotation field of your PROTO file or Webots VRML robot string.')
    optParser.add_option('--init-pos', dest='initPos', default=None,
                         help='Set the initial positions of your robot joints. Example: --init-pos="[1.2, 0.5, -1.5]" would '
                         'set the first 3 joints of your robot to the specified values, and leave the rest with their '
                         'default value.')
    optParser.add_option('--link-to-def', dest='linkToDef', action='store_true', default=False,
                         help='If set, urdf link names are also used as DEF names as well as solid names.')
    optParser.add_option('--joint-to-def', dest='jointToDef', action='store_true', default=False,
                         help='If set, urdf joint names are also used as DEF names as well as joint names.')
    options, args = optParser.parse_args()
    convert2urdf(options.inFile, options.outFile, options.isProto, options.robotName, options.normal, options.boxCollision, options.disableMeshOptimization,
                 options.enableMultiFile, options.toolSlot, options.initTranslation, options.initRotation, options.initPos, options.linkToDef, options.jointToDef)
