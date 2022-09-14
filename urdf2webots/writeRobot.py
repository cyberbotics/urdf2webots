"""Import modules."""

import math
import numpy as np

from urdf2webots.math_utils import rotateVector, matrixFromRotation, multiplyMatrix, rotationFromMatrix

isProto = True
toolSlot = None
staticBase = False
robotName = ''
initPos = None
linkToDef = False
jointToDef = False
enableMultiFile = False
meshFilesPath = None
indexSolid = 0


class RGB():
    """RGB color object."""

    def __init__(self):
        """Initialization."""
        self.red = 0.5
        self.green = 0.5
        self.blue = 0.5


# ref: https://marcodiiga.github.io/rgba-to-rgb-conversion
def RGBA2RGB(RGBA_color, RGB_background=RGB()):
    """Convert RGBA to RGB expression."""
    alpha = RGBA_color.alpha

    new_color = RGB()
    new_color.red = (1 - alpha) * RGB_background.red + alpha * RGBA_color.red
    new_color.green = (1 - alpha) * RGB_background.green + alpha * RGBA_color.green
    new_color.blue = (1 - alpha) * RGB_background.blue + alpha * RGBA_color.blue

    return new_color


def header(robotFile, srcFile=None, protoName=None, tags=[]):
    """Specify VRML file header."""
    robotFile.write('#VRML_SIM R2022b utf8\n')
    robotFile.write('# license: Apache License 2.0\n')
    robotFile.write('# license url: http://www.apache.org/licenses/LICENSE-2.0\n')
    if tags:
        robotFile.write('# tags: %s\n' % ','.join(tags))
    if protoName:
        robotFile.write('# This is a proto file for Webots for the ' + protoName + '\n')
    if srcFile:
        robotFile.write('# Extracted from: ' + srcFile + '\n')
    else:
        robotFile.write('# Extracted from a URDF content string\n')
    robotFile.write('\n')


def declaration(robotFile, robotName, initTranslation, initRotation):
    """Prototype declaration."""
    spaces = ' ' * max(1, len(robotName) - 2)
    robotFile.write('PROTO ' + robotName + ' [\n')
    robotFile.write('  field  SFVec3f     translation     ' + initTranslation + '\n')
    robotFile.write('  field  SFRotation  rotation        ' + initRotation + '\n')
    robotFile.write('  field  SFString    name            "' + robotName + '"  # Is `Robot.name`.\n')
    robotFile.write('  field  SFString    controller      "void"' + spaces + '# Is `Robot.controller`.\n')
    robotFile.write('  field  MFString    controllerArgs  []    ' + spaces + '# Is `Robot.controllerArgs`.\n')
    robotFile.write('  field  SFString    customData      ""    ' + spaces + '# Is `Robot.customData`.\n')
    robotFile.write('  field  SFBool      supervisor      FALSE ' + spaces + '# Is `Robot.supervisor`.\n')
    robotFile.write('  field  SFBool      synchronization TRUE  ' + spaces + '# Is `Robot.synchronization`.\n')
    robotFile.write('  field  SFBool      selfCollision   FALSE ' + spaces + '# Is `Robot.selfCollision`.\n')
    if toolSlot:
        robotFile.write('  field  MFNode      toolSlot        []    ' + spaces +
                    '# Extend the robot with new nodes at the end of the arm.\n')
    robotFile.write(']\n')
    robotFile.write('{\n')


def URDFLink(robotFile, link, level, parentList,
             childList, linkList, jointList, sensorList, jointPosition=[0.0, 0.0, 0.0],
             jointRotation=[1.0, 0.0, 0.0, 0.0], boxCollision=False, normal=False,
             dummy=False, robot=False, endpoint=False, initTranslation='', initRotation=''):
    """Write a link iteratively."""
    indent = '  '
    haveChild = False
    if not isProto:
        defaultSolidName = ''
    if robot:
        robotFile.write(level * indent + 'Robot {\n')
        if isProto:
            robotFile.write((level + 1) * indent + 'translation IS translation\n')
            robotFile.write((level + 1) * indent + 'rotation IS rotation\n')
            robotFile.write((level + 1) * indent + 'controller IS controller\n')
            robotFile.write((level + 1) * indent + 'controllerArgs IS controllerArgs\n')
            robotFile.write((level + 1) * indent + 'customData IS customData\n')
            robotFile.write((level + 1) * indent + 'supervisor IS supervisor\n')
            robotFile.write((level + 1) * indent + 'synchronization IS synchronization\n')
            robotFile.write((level + 1) * indent + 'selfCollision IS selfCollision\n')
        else:
            robotFile.write((level + 1) * indent + 'translation ' + initTranslation + '\n')
            robotFile.write((level + 1) * indent + 'rotation ' + initRotation + '\n')
    else:
        if link.forceSensor:
            robotFile.write((' ' if endpoint else level * indent) + ('DEF ' + link.name + ' ' if linkToDef else '') + 'TouchSensor {\n')
            robotFile.write((level + 1) * indent + 'type "force-3d"\n')
            robotFile.write((level + 1) * indent + 'lookupTable []\n')
        else:
            robotFile.write((' ' if endpoint else level * indent) + ('DEF ' + link.name + ' ' if linkToDef else '') + 'Solid {\n')
            if not isProto:
                # need a unique name for every solid node for the robot string
                global indexSolid
                defaultSolidName = 'solid' + str(indexSolid)
                indexSolid += 1

        robotFile.write((level + 1) * indent + 'translation %lf %lf %lf\n' % (jointPosition[0],
                                                                          jointPosition[1],
                                                                          jointPosition[2]))
        robotFile.write((level + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (jointRotation[0],
                                                                           jointRotation[1],
                                                                           jointRotation[2],
                                                                           jointRotation[3]))
    if not dummy:  # dummy: case when link not defined but referenced (e.g. Atlas robot)
        # 1: export Shapes
        if link.visual:
            if not haveChild:
                haveChild = True
                robotFile.write((level + 1) * indent + 'children [\n')
            URDFShape(robotFile, link, level + 2, normal)
        # 2: export Sensors
        for sensor in sensorList:
            if sensor.parentLink == link.name:
                if not haveChild:
                    haveChild = True
                    robotFile.write((level + 1) * indent + 'children [\n')
                if hasattr(sensor, 'isImager') and sensor.isImager:
                    robotFile.write((level + 2) * indent + 'Transform {\n')
                    robotFile.write((level + 3) * indent + 'translation 0 0 0\n')
                    robotFile.write((level + 3) * indent + 'rotation 0.577350 -0.577350 0.577350 2.094395\n')
                    robotFile.write((level + 3) * indent + 'children [\n')
                    sensor.export(robotFile, level + 4)
                    robotFile.write((level + 3) * indent + ']\n')
                    robotFile.write((level + 2) * indent + '}\n')
                else:
                    sensor.export(robotFile, level + 2)
        # 3: export Joints
        for joint in jointList:
            if joint.parent == link.name:
                if not haveChild:
                    haveChild = True
                    robotFile.write((level + 1) * indent + 'children [\n')
                URDFJoint(robotFile, joint, level + 2, parentList, childList,
                          linkList, jointList, sensorList, boxCollision, normal)
        # 4: export ToolSlot if specified
        if link.name == toolSlot:
            if not haveChild:
                robotFile.write((level + 1) * indent + 'children [\n')
            robotFile.write((level + 2) * indent + 'Group {\n')
            robotFile.write((level + 3) * indent + 'children IS toolSlot\n')
            robotFile.write((level + 2) * indent + '}\n')
            robotFile.write((level + 1) * indent + ']\n')
            # add dummy physics and bounding object, so tools don't fall off
            if link.inertia.mass is None:
                robotFile.write((level + 1) * indent + 'physics Physics {\n')
                robotFile.write((level + 1) * indent + '}\n')
                robotFile.write((level + 1) * indent + 'boundingObject Box {\n')
                robotFile.write((level + 2) * indent + 'size 0.01 0.01 0.01\n')
                robotFile.write((level + 1) * indent + '}\n')
        elif haveChild:
            robotFile.write((level + 1) * indent + ']\n')
        if isProto:
            if level == 1:
                robotFile.write((level + 1) * indent + 'name IS name\n')
            else:
                robotFile.write((level + 1) * indent + 'name "' + link.name + '"\n')
        elif defaultSolidName:
            robotFile.write((level + 1) * indent + 'name "' + defaultSolidName + '"\n')

        if link.collision:
            URDFBoundingObject(robotFile, link, level + 1, boxCollision)
        if link.inertia.mass is not None:
            if isProto:
                if level > 1 or not staticBase:
                    writeLinkPhysics(robotFile, link, level)
            else:
                if level != 0 or not staticBase:
                    writeLinkPhysics(robotFile, link, level)
        elif link.collision:
            if isProto:
                if level > 1 or not staticBase:
                    robotFile.write((level + 1) * indent + 'physics Physics {\n')
                    robotFile.write((level + 1) * indent + '}\n')
            else:
                if level != 0 or not staticBase:
                    robotFile.write((level + 1) * indent + 'physics Physics {\n')
                    robotFile.write((level + 1) * indent + '}\n')
    if not isProto:
        if robot:
            robotFile.write((level + 1) * indent + 'name "'+robotName+'"\n')
            robotFile.write((level + 1) * indent + 'controller "<extern>"\n')
    robotFile.write(level * indent + '}\n')


def writeLinkPhysics(robotFile, link, level):
    """Write a Webots Physics node."""
    indent = '  '

    robotFile.write((level + 1) * indent + 'physics Physics {\n')
    robotFile.write((level + 2) * indent + 'density -1\n')
    robotFile.write((level + 2) * indent + 'mass %lf\n' % link.inertia.mass)
    robotFile.write((level + 2) * indent + 'centerOfMass [ %lf %lf %lf ]\n' % (link.inertia.position[0],
                                                                        link.inertia.position[1],
                                                                        link.inertia.position[2]))
    if link.inertia.ixx > 0.0 and link.inertia.iyy > 0.0 and link.inertia.izz > 0.0:
        i = link.inertia
        inertiaMatrix = [i.ixx, i.ixy, i.ixz, i.ixy, i.iyy, i.iyz, i.ixz, i.iyz, i.izz]
        if link.inertia.rotation[-1] != 0.0:
            rotationMatrix = matrixFromRotation(link.inertia.rotation)
            I_mat = np.array(inertiaMatrix).reshape(3, 3)
            R = np.array(rotationMatrix).reshape(3, 3)
            R_t = np.transpose(R)
            # calculate the rotated inertiaMatrix with R_t * I * R. For reference, check the link below
            # https://www.euclideanspace.com/physics/dynamics/inertia/rotation/index.htm
            inertiaMatrix = np.dot(np.dot(R_t, I_mat), R).reshape(9)
        if (inertiaMatrix[0] != 1.0 or inertiaMatrix[4] != 1.0 or inertiaMatrix[8] != 1.0 or
                inertiaMatrix[1] != 0.0 or inertiaMatrix[2] != 0.0 or inertiaMatrix[5] != 0.0):
            robotFile.write((level + 2) * indent + 'inertiaMatrix [\n')
            # principals moments of inertia (diagonal)
            robotFile.write((level + 3) * indent + '%e %e %e\n' % (inertiaMatrix[0], inertiaMatrix[4], inertiaMatrix[8]))
            # products of inertia
            robotFile.write((level + 3) * indent + '%e %e %e\n' % (inertiaMatrix[1], inertiaMatrix[2], inertiaMatrix[5]))
            robotFile.write((level + 2) * indent + ']\n')
    robotFile.write((level + 1) * indent + '}\n')


def URDFBoundingObject(robotFile, link, level, boxCollision):
    """Write an boundingObject."""
    indent = '  '
    boundingLevel = level
    robotFile.write(level * indent + 'boundingObject ')
    hasGroup = len(link.collision) > 1
    if hasGroup:
        robotFile.write('Group {\n')
        robotFile.write((level + 1) * indent + 'children [\n')
        boundingLevel = level + 2

    for boundingObject in link.collision:
        initialIndent = boundingLevel * indent if hasGroup else ''
        if not boxCollision and boundingObject.position != [0.0, 0.0, 0.0] or boundingObject.rotation[3] != 0.0:
            robotFile.write(initialIndent + 'Transform {\n')
            robotFile.write((boundingLevel + 1) * indent + 'translation %lf %lf %lf\n' % (boundingObject.position[0],
                                                                                      boundingObject.position[1],
                                                                                      boundingObject.position[2]))
            robotFile.write((boundingLevel + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (boundingObject.rotation[0],
                                                                                       boundingObject.rotation[1],
                                                                                       boundingObject.rotation[2],
                                                                                       boundingObject.rotation[3]))
            robotFile.write((boundingLevel + 1) * indent + 'children [\n')
            boundingLevel = boundingLevel + 2
            hasGroup = True
            initialIndent = boundingLevel * indent

        if boundingObject.geometry.box.x != 0:
            robotFile.write(initialIndent + 'Box {\n')
            robotFile.write((boundingLevel + 1) * indent + ' size %lf %lf %lf\n' % (boundingObject.geometry.box.x,
                                                                                boundingObject.geometry.box.y,
                                                                                boundingObject.geometry.box.z))
            robotFile.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.cylinder.radius != 0 and boundingObject.geometry.cylinder.length != 0:
            robotFile.write(initialIndent + 'Cylinder {\n')
            robotFile.write((boundingLevel + 1) * indent + 'radius ' + str(boundingObject.geometry.cylinder.radius) + '\n')
            robotFile.write((boundingLevel + 1) * indent + 'height ' + str(boundingObject.geometry.cylinder.length) + '\n')
            robotFile.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.sphere.radius != 0:
            robotFile.write(initialIndent + 'Sphere {\n')
            robotFile.write((boundingLevel + 1) * indent + 'radius ' + str(boundingObject.geometry.sphere.radius) + '\n')
            robotFile.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.trimesh.coord and boxCollision:
            aabb = {
                'minimum': {'x': float('inf'),
                            'y': float('inf'),
                            'z': float('inf')},
                'maximum': {'x': float('-inf'),
                            'y': float('-inf'),
                            'z': float('-inf')}
            }
            for value in boundingObject.geometry.trimesh.coord:
                x = value[0] * boundingObject.geometry.scale[0]
                y = value[1] * boundingObject.geometry.scale[1]
                z = value[2] * boundingObject.geometry.scale[2]
                aabb['minimum']['x'] = min(aabb['minimum']['x'], x)
                aabb['maximum']['x'] = max(aabb['maximum']['x'], x)
                aabb['minimum']['y'] = min(aabb['minimum']['y'], y)
                aabb['maximum']['y'] = max(aabb['maximum']['y'], y)
                aabb['minimum']['z'] = min(aabb['minimum']['z'], z)
                aabb['maximum']['z'] = max(aabb['maximum']['z'], z)

            robotFile.write(initialIndent + 'Transform {\n')
            robotFile.write((boundingLevel + 1) * indent + 'translation %f %f %f\n' % (
                        0.5 * (aabb['maximum']['x'] + aabb['minimum']['x']) + boundingObject.position[0],
                        0.5 * (aabb['maximum']['y'] + aabb['minimum']['y']) + boundingObject.position[1],
                        0.5 * (aabb['maximum']['z'] + aabb['minimum']['z']) + boundingObject.position[2],))
            robotFile.write((boundingLevel + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (boundingObject.rotation[0],
                                                                                       boundingObject.rotation[1],
                                                                                       boundingObject.rotation[2],
                                                                                       boundingObject.rotation[3]))
            robotFile.write((boundingLevel + 1) * indent + 'children [\n')
            robotFile.write((boundingLevel + 2) * indent + 'Box {\n')
            robotFile.write((boundingLevel + 3) * indent + 'size %f %f %f\n' % (
                        aabb['maximum']['x'] - aabb['minimum']['x'],
                        aabb['maximum']['y'] - aabb['minimum']['y'],
                        aabb['maximum']['z'] - aabb['minimum']['z'],))
            robotFile.write((boundingLevel + 2) * indent + '}\n')
            robotFile.write((boundingLevel + 1) * indent + ']\n')
            robotFile.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.trimesh.coord:
            if boundingObject.geometry.defName is not None:
                robotFile.write(initialIndent + 'USE %s\n' % boundingObject.geometry.defName)
            else:
                if boundingObject.geometry.name is not None:
                    boundingObject.geometry.defName = computeDefName(boundingObject.geometry.name)
                    robotFile.write(initialIndent + 'DEF %s IndexedFaceSet {\n' % boundingObject.geometry.defName)
                else:
                    robotFile.write(initialIndent + 'IndexedFaceSet {\n')

                robotFile.write((boundingLevel + 1) * indent + 'coord Coordinate {\n')
                robotFile.write((boundingLevel + 2) * indent + 'point [\n' + (boundingLevel + 3) * indent)
                for value in boundingObject.geometry.trimesh.coord:
                    robotFile.write('%lf %lf %lf, ' % (value[0] * boundingObject.geometry.scale[0],
                                                   value[1] * boundingObject.geometry.scale[1],
                                                   value[2] * boundingObject.geometry.scale[2]))
                robotFile.write('\n' + (boundingLevel + 2) * indent + ']\n')
                robotFile.write((boundingLevel + 1) * indent + '}\n')

                robotFile.write((boundingLevel + 1) * indent + 'coordIndex [\n' + (boundingLevel + 2) * indent)
                if isinstance(boundingObject.geometry.trimesh.coordIndex[0], np.ndarray) \
                   or type(boundingObject.geometry.trimesh.coordIndex[0]) == list:
                    for value in boundingObject.geometry.trimesh.coordIndex:
                        if len(value) == 3:
                            robotFile.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
                elif isinstance(boundingObject.geometry.trimesh.coordIndex[0], np.int32):
                    for i in range(len(boundingObject.geometry.trimesh.coordIndex) / 3):
                        robotFile.write('%d %d %d -1 ' % (boundingObject.geometry.trimesh.coordIndex[3 * i + 0],
                                                      boundingObject.geometry.trimesh.coordIndex[3 * i + 1],
                                                      boundingObject.geometry.trimesh.coordIndex[3 * i + 2]))
                else:
                    print('Unsupported "%s" coordinate type' % type(boundingObject.geometry.trimesh.coordIndex[0]))
                robotFile.write('\n' + (boundingLevel + 1) * indent + ']\n')
                robotFile.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.mesh.url:
            if boundingObject.geometry.defName is not None:
                robotFile.write(initialIndent + 'USE %s\n' % boundingObject.geometry.defName)
            else:
                if boundingObject.geometry.name is not None:
                    boundingObject.geometry.defName = computeDefName(boundingObject.geometry.name)
                if boundingObject.geometry.defName is not None:
                    robotFile.write(initialIndent + 'DEF %s Mesh {\n' % boundingObject.geometry.defName)
                else:
                    robotFile.write(initialIndent + 'Mesh {\n')

                robotFile.write((boundingLevel + 1) * indent + 'url ' + str(boundingObject.geometry.mesh.url) + '\n')
                robotFile.write(boundingLevel * indent + '}\n')

        else:
            robotFile.write(initialIndent + 'Box{\n')
            robotFile.write((boundingLevel + 1) * indent + ' size 0.01 0.01 0.01\n')
            robotFile.write(boundingLevel * indent + '}\n')

        if boundingLevel == level + 4:
            robotFile.write((level + 3) * indent + ']\n')
            robotFile.write((level + 2) * indent + '}\n')
            boundingLevel = level + 2
    if boundingLevel == level + 2:
        robotFile.write((level + 1) * indent + ']\n')
        robotFile.write(level * indent + '}\n')


def computeDefName(name):
    """Compute a VRML compliant DEF name from an arbitrary string."""
    defName = name.replace(' ', '_').replace('.', '_')
    if not defName:  # empty string
        return None
    return name.replace(' ', '_').replace('.', '_')


def URDFVisual(robotFile, visualNode, level, normal=False):
    """Write a Visual."""
    indent = '  '
    shapeLevel = level

    robotFile.write(shapeLevel * indent + 'Shape {\n')
    if visualNode.material.defName is not None:
        robotFile.write((shapeLevel + 1) * indent + 'appearance USE %s\n' % visualNode.material.defName)
    else:
        if visualNode.material.name is not None:
            visualNode.material.defName = computeDefName(visualNode.material.name)
        if visualNode.geometry.lineset:
            if visualNode.material.defName is not None:
                robotFile.write((shapeLevel + 1) * indent + 'appearance DEF %s Appearance {\n' % visualNode.material.defName)
            else:
                robotFile.write((shapeLevel + 1) * indent + 'appearance Appearance {\n')

            robotFile.write((shapeLevel + 2) * indent + 'material Material {\n')

            ambientColor = RGBA2RGB(visualNode.material.ambient)
            diffuseColor = RGBA2RGB(visualNode.material.diffuse, RGB_background=ambientColor)
            emissiveColor = RGBA2RGB(visualNode.material.emission, RGB_background=ambientColor)
            specularColor = RGBA2RGB(visualNode.material.specular, RGB_background=ambientColor)

            robotFile.write((shapeLevel + 3) * indent + 'diffuseColor %lf %lf %lf\n' % (diffuseColor.red,
                                                                                diffuseColor.green,
                                                                                diffuseColor.blue))
            robotFile.write((shapeLevel + 3) * indent + 'emissiveColor %lf %lf %lf\n' % (emissiveColor.red,
                                                                                    emissiveColor.green,
                                                                                    emissiveColor.blue))
            if visualNode.material.shininess:
                robotFile.write((shapeLevel + 3) * indent + 'shininess %lf\n' % (visualNode.material.shininess))
            robotFile.write((shapeLevel + 3) * indent + 'specularColor %lf %lf %lf\n' % (specularColor.red,
                                                                                    specularColor.green,
                                                                                    specularColor.blue))
            robotFile.write((shapeLevel + 3) * indent + 'transparency %lf\n' % (1.0 - visualNode.material.diffuse.alpha))
            robotFile.write((shapeLevel + 2) * indent + '}\n')
            robotFile.write((shapeLevel + 1) * indent + '}\n')
        else:
            if visualNode.material.defName is not None:
                robotFile.write((shapeLevel + 1) * indent + 'appearance DEF %s PBRAppearance {\n' % visualNode.material.defName)
            else:
                robotFile.write((shapeLevel + 1) * indent + 'appearance PBRAppearance {\n')
            ambientColor = RGBA2RGB(visualNode.material.ambient)
            diffuseColor = RGBA2RGB(visualNode.material.diffuse, RGB_background=ambientColor)
            emissiveColor = RGBA2RGB(visualNode.material.emission, RGB_background=ambientColor)
            roughness = 1.0 - visualNode.material.specular.alpha * (visualNode.material.specular.red +
                                                                    visualNode.material.specular.green +
                                                                    visualNode.material.specular.blue) / 3.0
            if visualNode.material.shininess:
                roughness *= (1.0 - 0.5 * visualNode.material.shininess)
            robotFile.write((shapeLevel + 2) * indent + 'baseColor %lf %lf %lf\n' % (diffuseColor.red,
                                                                                diffuseColor.green,
                                                                                diffuseColor.blue))
            robotFile.write((shapeLevel + 2) * indent + 'transparency %lf\n' % (1.0 - visualNode.material.diffuse.alpha))
            robotFile.write((shapeLevel + 2) * indent + 'roughness %lf\n' % roughness)
            robotFile.write((shapeLevel + 2) * indent + 'metalness 0\n')
            robotFile.write((shapeLevel + 2) * indent + 'emissiveColor %lf %lf %lf\n' % (emissiveColor.red,
                                                                                    emissiveColor.green,
                                                                                    emissiveColor.blue))
            if visualNode.material.texture != "":
                robotFile.write((shapeLevel + 2) * indent + 'baseColorMap ImageTexture {\n')
                robotFile.write((shapeLevel + 3) * indent + 'url [ "' + visualNode.material.texture + '" ]\n')
                robotFile.write((shapeLevel + 2) * indent + '}\n')
            robotFile.write((shapeLevel + 1) * indent + '}\n')

    if visualNode.geometry.box.x != 0:
        robotFile.write((shapeLevel + 1) * indent + 'geometry Box {\n')
        robotFile.write((shapeLevel + 2) * indent + ' size ' +
                    str(visualNode.geometry.box.x) + ' ' +
                    str(visualNode.geometry.box.y) + ' ' +
                    str(visualNode.geometry.box.z) + '\n')
        robotFile.write((shapeLevel + 1) * indent + '}\n')

    elif visualNode.geometry.cylinder.radius != 0:
        robotFile.write((shapeLevel + 1) * indent + 'geometry Cylinder {\n')
        robotFile.write((shapeLevel + 2) * indent + 'radius ' + str(visualNode.geometry.cylinder.radius) + '\n')
        robotFile.write((shapeLevel + 2) * indent + 'height ' + str(visualNode.geometry.cylinder.length) + '\n')
        robotFile.write((shapeLevel + 1) * indent + '}\n')

    elif visualNode.geometry.sphere.radius != 0:
        robotFile.write((shapeLevel + 1) * indent + 'geometry Sphere {\n')
        robotFile.write((shapeLevel + 2) * indent + 'radius ' + str(visualNode.geometry.sphere.radius) + '\n')
        robotFile.write((shapeLevel + 1) * indent + '}\n')

    elif visualNode.geometry.trimesh.coord:
        meshType = 'IndexedLineSet' if visualNode.geometry.lineset else 'IndexedFaceSet'
        if visualNode.geometry.defName is not None:
            robotFile.write((shapeLevel + 1) * indent + 'geometry USE %s\n' % visualNode.geometry.defName)
        else:
            if visualNode.geometry.name is not None:
                visualNode.geometry.defName = computeDefName(visualNode.geometry.name)
            if visualNode.geometry.defName is not None:
                robotFile.write((shapeLevel + 1) * indent + 'geometry DEF %s %s {\n' % (visualNode.geometry.defName, meshType))
            else:
                robotFile.write((shapeLevel + 1) * indent + 'geometry %s {\n' % meshType)
            robotFile.write((shapeLevel + 2) * indent + 'coord Coordinate {\n')
            robotFile.write((shapeLevel + 3) * indent + 'point [\n' + (shapeLevel + 4) * indent)
            for value in visualNode.geometry.trimesh.coord:
                robotFile.write('%lf %lf %lf, ' % (value[0] * visualNode.geometry.scale[0],
                                               value[1] * visualNode.geometry.scale[1],
                                               value[2] * visualNode.geometry.scale[2]))
            robotFile.write('\n' + (shapeLevel + 3) * indent + ']\n')
            robotFile.write((shapeLevel + 2) * indent + '}\n')

            robotFile.write((shapeLevel + 2) * indent + 'coordIndex [\n' + (shapeLevel + 3) * indent)
            if (isinstance(visualNode.geometry.trimesh.coordIndex[0], np.ndarray) or
                    type(visualNode.geometry.trimesh.coordIndex[0]) == list):
                for value in visualNode.geometry.trimesh.coordIndex:
                    if len(value) == 3:
                        robotFile.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
                    elif len(value) == 2:
                        assert visualNode.geometry.lineset
                        robotFile.write('%d %d -1 ' % (value[0], value[1]))
            elif isinstance(visualNode.geometry.trimesh.coordIndex[0], np.int32):
                for i in range(int(len(visualNode.geometry.trimesh.coordIndex) / 3)):
                    robotFile.write('%d %d %d -1 ' % (visualNode.geometry.trimesh.coordIndex[3 * i + 0],
                                                  visualNode.geometry.trimesh.coordIndex[3 * i + 1],
                                                  visualNode.geometry.trimesh.coordIndex[3 * i + 2]))
            else:
                print('Unsupported "%s" coordinate type' % type(visualNode.geometry.trimesh.coordIndex[0]))
            robotFile.write('\n' + (shapeLevel + 2) * indent + ']\n')

            if normal and visualNode.geometry.trimesh.normal and visualNode.geometry.trimesh.normalIndex:
                robotFile.write((shapeLevel + 2) * indent + 'normal Normal {\n')
                robotFile.write((shapeLevel + 3) * indent + 'vector [\n' + (shapeLevel + 4) * indent)
                for value in visualNode.geometry.trimesh.normal:
                    robotFile.write('%lf %lf %lf, ' % (value[0], value[1], value[2]))
                robotFile.write('\n' + (shapeLevel + 3) * indent + ']\n')
                robotFile.write((shapeLevel + 2) * indent + '}\n')

                robotFile.write((shapeLevel + 2) * indent + 'normalIndex [\n' + (shapeLevel + 3) * indent)
                if (isinstance(visualNode.geometry.trimesh.normalIndex[0], np.ndarray) or
                        type(visualNode.geometry.trimesh.normalIndex[0]) == list):
                    for value in visualNode.geometry.trimesh.normalIndex:
                        if len(value) == 3:
                            robotFile.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
                elif isinstance(visualNode.geometry.trimesh.normalIndex[0], np.int32):
                    for i in range(len(visualNode.geometry.trimesh.normalIndex) / 3):
                        robotFile.write('%d %d %d -1 ' % (visualNode.geometry.trimesh.normalIndex[3 * i + 0],
                                                      visualNode.geometry.trimesh.normalIndex[3 * i + 1],
                                                      visualNode.geometry.trimesh.normalIndex[3 * i + 2]))
                else:
                    print('Unsupported "%s" normal type' % type(visualNode.geometry.trimesh.normalIndex[0]))
                robotFile.write('\n' + (shapeLevel + 2) * indent + ']\n')

            if visualNode.geometry.trimesh.texCoord:
                robotFile.write((shapeLevel + 2) * indent + 'texCoord TextureCoordinate {\n')
                robotFile.write((shapeLevel + 3) * indent + 'point [\n' + (shapeLevel + 4) * indent)
                for value in visualNode.geometry.trimesh.texCoord:
                    robotFile.write('%lf %lf, ' % (value[0], value[1]))
                robotFile.write('\n' + (shapeLevel + 3) * indent + ']\n')
                robotFile.write((shapeLevel + 2) * indent + '}\n')

                robotFile.write((shapeLevel + 2) * indent + 'texCoordIndex [\n' + (shapeLevel + 3) * indent)
                if (isinstance(visualNode.geometry.trimesh.texCoordIndex[0], np.ndarray) or
                        type(visualNode.geometry.trimesh.texCoordIndex[0]) == list):
                    for value in visualNode.geometry.trimesh.texCoordIndex:
                        if len(value) == 3:
                            robotFile.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
                elif isinstance(visualNode.geometry.trimesh.texCoordIndex[0], np.int32):
                    for i in range(len(visualNode.geometry.trimesh.texCoordIndex) // 3):
                        robotFile.write('%d %d %d -1 ' % (visualNode.geometry.trimesh.texCoordIndex[3 * i + 0],
                                                      visualNode.geometry.trimesh.texCoordIndex[3 * i + 1],
                                                      visualNode.geometry.trimesh.texCoordIndex[3 * i + 2]))
                else:
                    print('Unsupported "%s" coordinate type' % type(visualNode.geometry.trimesh.texCoordIndex[0]))
                robotFile.write('\n' + (shapeLevel + 2) * indent + ']\n')

            if not visualNode.geometry.lineset:
                robotFile.write((shapeLevel + 2) * indent + 'creaseAngle 1\n')
            robotFile.write((shapeLevel + 1) * indent + '}\n')

    elif visualNode.geometry.mesh.url:
        if visualNode.geometry.defName is not None:
            robotFile.write((shapeLevel + 1) * indent + 'geometry USE %s\n' % visualNode.geometry.defName)
        else:
            if visualNode.geometry.name is not None:
                visualNode.geometry.defName = computeDefName(visualNode.geometry.name)
            if visualNode.geometry.defName is not None:
                robotFile.write((shapeLevel + 1) * indent + 'geometry DEF %s Mesh {\n' % visualNode.geometry.defName)
            else:
                robotFile.write((shapeLevel + 1) * indent + 'geometry Mesh {\n')

            robotFile.write((shapeLevel + 2) * indent + 'url ' + str(visualNode.geometry.mesh.url) + '\n')
            robotFile.write((shapeLevel + 1) * indent + '}\n')

    robotFile.write(shapeLevel * indent + '}\n')


def URDFShape(robotFile, link, level, normal=False):
    """Write a Shape."""
    indent = '  '
    shapeLevel = level
    transform = False

    for visualNode in link.visual:
        if visualNode.position != [0.0, 0.0, 0.0] or visualNode.rotation[3] != 0:
            robotFile.write(shapeLevel * indent + 'Transform {\n')
            robotFile.write((shapeLevel + 1) * indent + 'translation %lf %lf %lf\n' % (visualNode.position[0],
                                                                                   visualNode.position[1],
                                                                                   visualNode.position[2]))
            robotFile.write((shapeLevel + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (visualNode.rotation[0],
                                                                                    visualNode.rotation[1],
                                                                                    visualNode.rotation[2],
                                                                                    visualNode.rotation[3]))
            robotFile.write((shapeLevel + 1) * indent + 'children [\n')
            shapeLevel += 2
            transform = True
        if enableMultiFile and visualNode.geometry.trimesh.coord:
            name = visualNode.geometry.defName
            if name is None:
                if visualNode.geometry.name is not None:
                    name = computeDefName(visualNode.geometry.name)
            name = robotName + '_' + name if robotName else name
            if visualNode.geometry.defName is None:
                print('Create meshFile: %sMesh.proto' % name)
                filepath = '%s/%sMesh.proto' % (meshFilesPath, name)
                meshProtoFile = open(filepath, 'w')
                header(meshProtoFile, tags=['hidden'])
                meshProtoFile.write('PROTO %sMesh [\n]\n{\n' % name)
                visualNode.material.defName = None
                URDFVisual(meshProtoFile, visualNode, 1, normal)
                meshProtoFile.write('}\n')
                meshProtoFile.close()
            robotFile.write(shapeLevel * indent + '%sMesh {\n' % name + shapeLevel * indent + '}\n')
        else:
            URDFVisual(robotFile, visualNode, shapeLevel, normal)
        if transform:
            robotFile.write((shapeLevel - 1) * indent + ']\n')
            robotFile.write((shapeLevel - 2) * indent + '}\n')
            shapeLevel -= 2


def URDFJoint(robotFile, joint, level, parentList, childList, linkList, jointList,
              sensorList, boxCollision, normal):
    """Write a Joint iteratively."""
    indent = '  '
    if not joint.axis:
        joint.axis = [1, 0, 0]
    axis = joint.axis
    endpointRotation = joint.rotation
    endpointPosition = joint.position
    if joint.rotation[3] != 0.0 and axis:
        axis = rotateVector(axis, joint.rotation)
    if joint.type == 'revolute' or joint.type == 'continuous':
        robotFile.write(level * indent + ('DEF ' + joint.name + ' ' if jointToDef else '') + 'HingeJoint {\n')
        robotFile.write((level + 1) * indent + 'jointParameters HingeJointParameters {\n')
        position = None
        if joint.limit.lower > 0.0:
            # if 0 is not in the range, set the position to be the middle of the range
            position = joint.limit.lower
            if joint.limit.upper >= joint.limit.lower:
                position = (joint.limit.upper - joint.limit.lower) / 2.0 + joint.limit.lower
        elif joint.limit.upper < 0.0:
            # if 0 is not in the range, set the position to be the middle of the range
            position = joint.limit.upper
            if joint.limit.upper >= joint.limit.lower:
                position = (joint.limit.upper - joint.limit.lower) / 2.0 + joint.limit.lower
        if initPos is not None:
            if len(initPos) > 0:
                position = initPos[0]
                del initPos[0]
        if position is not None:
            robotFile.write((level + 2) * indent + 'position %lf\n' % position)
            mat1 = matrixFromRotation(endpointRotation)
            mat2 = matrixFromRotation([axis[0], axis[1], axis[2], position])
            mat3 = multiplyMatrix(mat2, mat1)
            endpointRotation = rotationFromMatrix(mat3)
        robotFile.write((level + 2) * indent + 'axis %lf %lf %lf\n' % (axis[0], axis[1], axis[2]))
        robotFile.write((level + 2) * indent + 'anchor %lf %lf %lf\n' % (joint.position[0], joint.position[1], joint.position[2]))
        robotFile.write((level + 2) * indent + 'dampingConstant ' + str(joint.dynamics.damping) + '\n')
        robotFile.write((level + 2) * indent + 'staticFriction ' + str(joint.dynamics.friction) + '\n')
        robotFile.write((level + 1) * indent + '}\n')
        robotFile.write((level + 1) * indent + 'device [\n')
        robotFile.write((level + 2) * indent + 'RotationalMotor {\n')
    elif joint.type == 'prismatic':
        robotFile.write(level * indent + ('DEF ' + joint.name + ' ' if jointToDef else '') + 'SliderJoint {\n')
        robotFile.write((level + 1) * indent + 'jointParameters JointParameters {\n')
        position = None
        if joint.limit.lower > 0.0:
            # if 0 is not in the range, set the position to be the middle of the range
            position = joint.limit.lower
            if joint.limit.upper >= joint.limit.lower:
                position = (joint.limit.upper - joint.limit.lower) / 2.0 + joint.limit.lower
        if initPos is not None:
            if len(initPos) > 0:
                position = initPos[0]
                del initPos[0]
        if position is not None:
            robotFile.write((level + 2) * indent + 'position %lf\n' % position)
            length = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
            if length > 0:
                endpointPosition[0] += axis[0] / length * position
                endpointPosition[1] += axis[1] / length * position
                endpointPosition[2] += axis[2] / length * position
        robotFile.write((level + 2) * indent + 'axis %lf %lf %lf\n' % (axis[0], axis[1], axis[2]))
        robotFile.write((level + 2) * indent + 'dampingConstant ' + str(joint.dynamics.damping) + '\n')
        robotFile.write((level + 2) * indent + 'staticFriction ' + str(joint.dynamics.friction) + '\n')
        robotFile.write((level + 1) * indent + '}\n')
        robotFile.write((level + 1) * indent + 'device [\n')
        robotFile.write((level + 2) * indent + 'LinearMotor {\n')
    elif joint.type == 'fixed':
        for childLink in linkList:
            if childLink.name == joint.child:
                URDFLink(robotFile, childLink, level, parentList, childList,
                         linkList, jointList, sensorList, joint.position, joint.rotation,
                         boxCollision, normal)
        return

    elif joint.type == 'floating' or joint.type == 'planar':
        print(joint.type + ' is not a supported joint type in Webots')
        return

    robotFile.write((level + 3) * indent + 'name "' + joint.name + '"\n')
    if joint.limit.velocity != 0.0:
        robotFile.write((level + 3) * indent + 'maxVelocity ' + str(joint.limit.velocity) + '\n')
    if joint.limit.lower != 0.0:
        robotFile.write((level + 3) * indent + 'minPosition ' + str(joint.limit.lower) + '\n')
    if joint.limit.upper != 0.0:
        robotFile.write((level + 3) * indent + 'maxPosition ' + str(joint.limit.upper) + '\n')
    if joint.limit.effort != 0.0:
        if joint.type == 'prismatic':
            robotFile.write((level + 3) * indent + 'maxForce ' + str(joint.limit.effort) + '\n')
        else:
            robotFile.write((level + 3) * indent + 'maxTorque ' + str(joint.limit.effort) + '\n')
    robotFile.write((level + 2) * indent + '}\n')
    robotFile.write((level + 2) * indent + 'PositionSensor {\n')
    robotFile.write((level + 3) * indent + 'name "' + joint.name + '_sensor"\n')
    robotFile.write((level + 2) * indent + '}\n')
    robotFile.write((level + 1) * indent + ']\n')

    robotFile.write((level + 1) * indent + 'endPoint')
    found_link = False
    for childLink in linkList:
        if childLink.name == joint.child:
            URDFLink(robotFile, childLink, level + 1, parentList, childList,
                     linkList, jointList, sensorList, endpointPosition, endpointRotation,
                     boxCollision, normal, endpoint=True)
            assert(not found_link)
            found_link = True
    # case that non-existing link cited, set dummy flag
    if not found_link and joint.child:
        URDFLink(robotFile, joint.child, level + 1, parentList, childList,
                 linkList, jointList, sensorList, endpointPosition, endpointRotation,
                 boxCollision, normal, dummy=True)
        print('warning: link ' + joint.child + ' is dummy!')
    robotFile.write(level * indent + '}\n')
