#! /usr/bin/env python

import getopt
import os, errno
import sys

import parserURDF
import writeProto
import xml #@ 

# import trainingSDF
# from xacro import xacro

from xml.dom import minidom


def usage():
    """Display command usage on standard out stream."""
    print (sys.argv[0] + ' inputFile.urdf [-o outputFile] [--box-collision]')


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

domFile = minidom.parse(xmlFile)

# extension = os.path.splitext(xmlFile)[1]
# if extension == '.xacro':
#    xacro.main()

def convertLUtoUN(s):
  r = ''
  i = 0
  while i < len(s):
    if i == 0:
      r += s[i].upper()
      i += 1
    elif s[i] == '_' and i < (len(s) - 1):
      r += s[i+1].upper()
      i += 2
    else:
      r += s[i]
      i += 1
  return r

def mkdirSafe(directory): #@ create a dir safely
    try:
        os.makedirs(directory)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
        else:
            print 'dir', directory, 'already existing!'


for child in domFile.childNodes:
    '''
    if child.localName == 'gazebo':
        print ('this is a sdf file')
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
        print ('this is an urdf file')
    '''
    if child.localName == 'robot':
        robotName = convertLUtoUN( parserURDF.getRobotName(child) ) #@capitalized
        
        parserURDF.robotName = robotName #@ pass robotName
        mkdirSafe(robotName+'_textures') #@ make a dir called 'x_textures'
        
        #protoFile = os.path.splitext(xmlFile)[0]
        protoFile = robotName #@ use robot name rather than urdf name (capitalized)
        robot = child
        protoFile = open(protoFile+'.proto', 'w')
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
                print ('root link is ' + rootLink.name)
        pluginList = parserURDF.getPlugins(robot)
        print ('there is ' + str(len(linkList)) + ' links, ' + str(len(jointList)) + ' joints and ' + str(len(pluginList)) + ' plugins')

        writeProto.URDFLink(protoFile, rootLink, 3, parentList, childList, linkList, jointList, boxCollision=boxCollision)
        protoFile.write('    ]\n')
        writeProto.basicPhysics(protoFile)
        protoFile.write('  }\n')
        protoFile.write('}\n')
        protoFile.close()
        exit(1)
print ('could not read file')
