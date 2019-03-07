#!/usr/bin/env python

import getopt
import os
import errno
import re
import sys
import parserURDF
import writeProto
from xml.dom import minidom


def usage():
    """Display command usage on standard out stream."""
    print(sys.argv[0] + ' inputFile.urdf [-o outputFile] [--box-collision]')


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
            print('Directory "' + directory + '" already existing!')


if len(sys.argv) < 2:
    usage()
    sys.exit(-1)

xmlFile = sys.argv[1]
argv = sys.argv[2:]
outputFile = os.path.splitext(xmlFile)[0] + '.proto'
boxCollision = False

try:
    opts, args = getopt.getopt(argv, "ho:", ["help", "box-collision"])
except getopt.GetoptError:
    usage()
    sys.exit(-1)
for opt, arg in opts:
    if opt in ("-h", "--help"):
        usage()
        sys.exit()
    elif opt == "-o":
        outputFile = arg
    elif opt == "--box-collision":
        boxCollision = True
    else:
        usage()
        sys.exit(-1)

with open(xmlFile, 'r') as file:
    content = file.read()

    packages = re.findall('"package://(.*)"', content)
    if packages:
        packageName = packages[0].split('/')[0]
        directory = os.path.dirname(xmlFile)
        while packageName != os.path.split(directory)[1] and os.path.split(directory)[1]:
            directory = os.path.dirname(directory)
        if os.path.split(directory)[1]:
            packagePath = os.path.split(directory)[0]
            content = content.replace('package:/', packagePath)
        else:
            sys.stderr.write("Can't determine package root path.\n")

    domFile = minidom.parseString(content)

    for child in domFile.childNodes:
        '''
        if child.localName == 'gazebo':
            print('this is a sdf file')
            robotName = trainingSDF.getModelName(domFile)
            protoFile = xmlFile.strip('.model')
            protoFile=open(protoFile+'.proto','w')
            writeProto.header(protoFile,xmlFile,robotName)
            writeProto.declaration(protoFile,robotName)
            for Node in domFile.getElementsByTagName('link'):
                writeProto.SDFLink(protoFile,Node)
            protoFile.write('       ]\n')
            protoFile.write('   }\n')
            protoFile.write('}\n')
            protoFile.close()
            exit(0)
        elif child.localName == 'robot':
            print('this is an urdf file')
        '''
        if child.localName == 'robot':
            robotName = convertLUtoUN(parserURDF.getRobotName(child))  # capitalize

            parserURDF.robotName = robotName  # pass robotName
            mkdirSafe(robotName + '_textures')  # make a dir called 'x_textures'

            protoFile = robotName             # use robot name rather than urdf name
            robot = child
            protoFile = open(protoFile + '.proto', 'w')
            writeProto.header(protoFile, xmlFile, robotName)
            writeProto.declaration(protoFile, robotName)
            linkElementList = []
            jointElementList = []
            for child in robot.childNodes:
                if child.localName == 'link':
                    linkElementList.append(child)
                elif child.localName == 'joint':
                    jointElementList.append(child)

            linkList = []
            jointList = []
            parentList = []
            childList = []
            rootLink = parserURDF.Link()

            for joint in jointElementList:
                jointList.append(parserURDF.getJoint(joint))
                parentList.append(jointList[-1].parent.encode("ascii"))
                childList.append(jointList[-1].child.encode("ascii"))
            parentList.sort()
            childList.sort()
            for link in linkElementList:
                linkList.append(parserURDF.getLink(link))
                if parserURDF.isRootLink(linkList[-1].name, childList):
                    rootLink = linkList[-1]
                    # if root link has only one joint which type is fixed,
                    # it should not be part of the model (link between robot and static environment)
                    directJoint = []
                    for joint in jointList:
                        if joint.parent == rootLink.name:
                            directJoint.append(joint)
                    if len(directJoint) == 1 and directJoint[0].type == 'fixed':
                        for childLink in linkList:
                            if childLink.name == directJoint[0].child:
                                rootLink = childLink
                                break
                    print('Root link: ' + rootLink.name)
            pluginList = parserURDF.getPlugins(robot)
            print('There are %d links, %d joints and %d plugins' % (len(linkList), len(jointList), len(pluginList)))

            writeProto.URDFLink(protoFile, rootLink, 1, parentList, childList, linkList, jointList,
                                boxCollision=boxCollision, robot=True)
            protoFile.write('}\n')
            protoFile.close()
            exit(1)
print('Could not read file')
