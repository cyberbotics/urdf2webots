#! /usr/bin/env python


import os
import string
import sys
import xml


def header(proto,srcFile, robotName):
    proto.write('#VRML_SIM V7.4.0 utf8\n')
    proto.write('# This is a proto file for Webots for the ' + robotName + '\n')
    proto.write('# Extracted from: ' + srcFile + '\n\n')

def declaration(proto, robotName):
    proto.write('PROTO '+robotName+' [\n')
    proto.write('   field SFVec3f translation 0 0 0\n')
    proto.write('   field SFRotation rotation 1 0 0 -1.57\n')
    proto.write('   field SFString controller "void"\n')
    proto.write(']\n')
    proto.write('{\n')
    proto.write('  Robot {\n')
    proto.write('    translation IS translation\n')
    proto.write('    rotation IS rotation\n')
    proto.write('    controller IS controller\n')
    proto.write('    children [\n')

def basicPhysics(proto):
    proto.write('    boundingObject Box{\n')
    proto.write('      size 0.01 0.01 0.01\n')
    proto.write('    }\n')
    proto.write('    physics Physics {\n')
    proto.write('    }\n')

def URDFLink(proto, link, level, parentList, childList, linkList, jointList, jointPosition = [0.0, 0.0, 0.0], jointRotation = [1.0, 0.0, 0.0, 0.0], boxCollision=False):
    indent = '  '
    haveChild = 0
    if link.collision:
        proto.write(level * indent + ' Solid {\n')
        proto.write((level + 1) * indent + 'name "' + link.name + '"\n')
    else:
        proto.write(level * indent + ' Transform {\n')

    proto.write((level + 1) * indent + 'translation ' + str(jointPosition[0]) + ' ' + str(jointPosition[1]) + ' ' + str(jointPosition[2]) + '\n')
    proto.write((level + 1) * indent + 'rotation ' + str(jointRotation[0]) + ' ' + str(jointRotation[1]) + ' ' + str(jointRotation[2]) + ' ' + str(jointRotation[3]) + '\n')

    if link.collision:
        proto.write((level + 1) * indent + 'physics Physics {\n')
        proto.write((level + 2) * indent + 'density -1\n')
        mass = link.inertia.mass
        # mass = min(100, max(0.1, link.inertia.mass)) # evt. clamp the masses
        proto.write((level + 2) * indent + 'mass ' + str(mass) + '\n')
        proto.write((level + 2) * indent + 'inertiaMatrix [' + str(link.inertia.ixx) + ' ' + str(link.inertia.iyy) + ' ' + str(link.inertia.izz) + ' ' + str(link.inertia.ixy) + ' ' + str(link.inertia.ixz) + ' ' + str(link.inertia.iyz) + ']\n')
        proto.write((level + 1) * indent + '}\n')
        URDFBoundingObject(proto, link, level + 1, boxCollision)

    if link.visual:
        proto.write((level + 1) * indent + 'children [\n')
        haveChild = 1
        URDFShape(proto, link, level + 2)

    for joint in jointList:
        if joint.parent == link.name:
            if haveChild == 0:
                haveChild = 1
                proto.write((level + 1) * indent + 'children [\n')
            URDFJoint(proto, joint, level + 2, parentList, childList, linkList, jointList, boxCollision)
    if haveChild == 1:
        proto.write((level + 1) * indent + ']\n')
    proto.write(level * indent + '}\n')


def URDFBoundingObject(proto, link, level, boxCollision):
    indent = '  '
    boundingLevel = level
    proto.write(level * indent + 'boundingObject ')
    if len(link.collision) > 1:
        proto.write('Group {\n')
        proto.write((level + 1) * indent + 'children [\n')
        boundingLevel = level + 2

    for boundingObject in link.collision:
        if boundingObject.position != [0.0, 0.0, 0.0] or  boundingObject.rotation[3] != 0.0:
            if len(link.collision) > 1:
                proto.write((level + 2) * indent + 'Transform {\n')
            else:
                proto.write('Transform {\n')
            proto.write((boundingLevel + 1) * indent + 'translation ' + str(boundingObject.position[0]) + ' ' + str(boundingObject.position[1]) + ' ' + str(boundingObject.position[2]) + '\n')
            proto.write((boundingLevel + 1) * indent + 'rotation ' + str(boundingObject.rotation[0]) + ' ' + str(boundingObject.rotation[1]) + ' ' + str(boundingObject.rotation[2]) + ' ' + str(boundingObject.rotation[3]) + '\n')
            proto.write((boundingLevel + 1) * indent + 'children [\n')
            boundingLevel = boundingLevel + 2

        if boundingObject.geometry.box.x != 0:
            proto.write(boundingLevel * indent + 'Box {\n')
            proto.write((boundingLevel + 1) * indent + ' size ' + str(boundingObject.geometry.box.x) + ' ' + str(boundingObject.geometry.box.y) + ' ' + str(boundingObject.geometry.box.z) + '\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.cylinder.radius != 0 and boundingObject.geometry.cylinder.length != 0:
            proto.write(boundingLevel * indent + 'Cylinder {\n')
            proto.write((boundingLevel + 1) * indent + 'radius ' + str(boundingObject.geometry.cylinder.radius) + '\n')
            proto.write((boundingLevel + 1) * indent + 'height ' + str(boundingObject.geometry.cylinder.length) + '\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.sphere.radius != 0:
            proto.write(boundingLevel * indent + 'Sphere {\n')
            proto.write((boundingLevel + 1) * indent + 'radius ' + str(boundingObject.geometry.sphere.radius) + '\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.trimesh.coord != [] and boxCollision:
            aabb = { 'minimum' : { 'x' : float('inf'), 'y' : float('inf'), 'z' : float('inf') }, \
                     'maximum' : { 'x' : float('-inf'), 'y' : float('-inf'), 'z' : float('-inf') }}
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

            proto.write(boundingLevel * indent + 'Transform {\n')
            proto.write((boundingLevel + 2) * indent + 'translation %f %f %f\n' % ( \
                0.5 * (aabb['maximum']['x'] + aabb['minimum']['x']), \
                0.5 * (aabb['maximum']['y'] + aabb['minimum']['y']), \
                0.5 * (aabb['maximum']['z'] + aabb['minimum']['z']), \
            ))
            proto.write((boundingLevel + 1) * indent + 'children [\n')
            proto.write((boundingLevel + 2) * indent + 'Box {\n')
            proto.write((boundingLevel + 3) * indent + 'size %f %f %f\n' % ( \
                aabb['maximum']['x'] - aabb['minimum']['x'], \
                aabb['maximum']['y'] - aabb['minimum']['y'], \
                aabb['maximum']['z'] - aabb['minimum']['z'], \
            ))
            proto.write((boundingLevel + 2) * indent + '}\n')
            proto.write((boundingLevel + 1) * indent + ']\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.trimesh.coord != []:
            proto.write(boundingLevel* indent + 'IndexedFaceSet {\n')

            proto.write((boundingLevel + 1) * indent + 'coord Coordinate {\n')
            proto.write((boundingLevel + 2) * indent + 'point [\n')
            for value in boundingObject.geometry.trimesh.coord:
                proto.write(boundingLevel * indent + str(value[0] * boundingObject.geometry.scale[0]) + ' ' + str(value[1] * boundingObject.geometry.scale[1]) + ' ' + str(value[2] * boundingObject.geometry.scale[2]) + '\n')
            proto.write((boundingLevel + 2) * indent + ']\n')
            proto.write((boundingLevel + 1) * indent + '}\n')

            proto.write((boundingLevel + 1) * indent + 'coordIndex [\n')
            for value in boundingObject.geometry.trimesh.coordIndex:
                proto.write(boundingLevel * indent + str(value[0]) + ' ' + str(value[1]) + ' ' + str(value[2]) + ' -1\n')
            proto.write((boundingLevel + 1) * indent + ']\n')

            proto.write((boundingLevel + 1) * indent + 'creaseAngle 0.5\n')
            proto.write(boundingLevel * indent + '}\n')

        else:
            proto.write((boundingLevel + 1) * indent + 'Box{\n')
            proto.write((boundingLevel + 1) * indent + ' size 0.01 0.01 0.01\n')
            proto.write(boundingLevel * indent + '}\n')

        if boundingLevel == level + 4:
            proto.write((level + 3) * indent + ']\n')
            proto.write((level + 2) * indent + '}\n')
            boundingLevel = level + 2
    if boundingLevel == level + 2:
        proto.write((level + 1) * indent + ']\n')
        proto.write(level * indent + '}\n')

def URDFShape(proto, link, level):
    indent = '  '
    shapeLevel = level
    transform = False
    group = False
    if len(link.visual) > 1:
        proto.write(level * indent + 'Group {\n')
        proto.write((level + 1) * indent + 'children [\n')
        shapeLevel = level + 2
        group = True

    for visualNode in link.visual:
        if visualNode.position != [0.0, 0.0, 0.0] or  visualNode.rotation[3] != 0:
            proto.write(shapeLevel * indent + 'Transform {\n')
            proto.write((shapeLevel + 1) * indent + 'translation ' + str(visualNode.position[0]) + ' ' + str(visualNode.position[1]) + ' ' + str(visualNode.position[2]) + '\n')
            proto.write((shapeLevel + 1) * indent + 'rotation ' + str(visualNode.rotation[0]) + ' ' + str(visualNode.rotation[1]) + ' ' + str(visualNode.rotation[2]) + ' ' + str(visualNode.rotation[3]) +'\n')
            proto.write((shapeLevel + 1) * indent + 'children [\n')
            shapeLevel = shapeLevel + 2
            transform = True

        proto.write(shapeLevel * indent + 'Shape {\n')
        proto.write((shapeLevel + 1) * indent + 'appearance Appearance {\n')
        proto.write((shapeLevel + 2) * indent + 'material Material {\n')
        if visualNode.material.color.red != 0 or visualNode.material.color.green != 0 or visualNode.material.color.blue != 0:
            proto.write((shapeLevel + 3) * indent + 'diffuseColor ' + str(visualNode.material.color.alpha * visualNode.material.color.red + 1 - visualNode.material.color.alpha) + ' ' + str(visualNode.material.color.alpha * visualNode.material.color.green + 1 - visualNode.material.color.alpha) + ' ' + str(visualNode.material.color.alpha * visualNode.material.color.blue + 1 - visualNode.material.color.alpha) + '\n')
        proto.write((shapeLevel + 2) * indent + '}\n')
        if visualNode.material.texture != "":
            proto.write((shapeLevel + 2) * indent + 'texture ImageTexture {\n')
            proto.write((shapeLevel + 3) * indent + 'url ["' + visualNode.material.texture + '"]\n')
            proto.write((shapeLevel + 2) * indent + '}\n')
        proto.write((shapeLevel + 1) * indent + '}\n')

        if visualNode.geometry.box.x != 0:
            proto.write((shapeLevel + 1) * indent + 'geometry Box {\n')
            proto.write((shapeLevel + 2) * indent + ' size ' + str(visualNode.geometry.box.x) + ' ' + str(visualNode.geometry.box.y) + ' ' + str(visualNode.geometry.box.z) + '\n')
            proto.write((shapeLevel + 1) * indent + '}\n')

        elif visualNode.geometry.cylinder.radius != 0:
            proto.write((shapeLevel + 1) * indent + 'geometry Cylinder {\n')
            proto.write((shapeLevel + 2) * indent + 'radius ' + str(visualNode.geometry.cylinder.radius) + '\n')
            proto.write((shapeLevel + 2) * indent + 'height ' + str(visualNode.geometry.cylinder.length) + '\n')
            proto.write((shapeLevel + 1) * indent + '}\n')

        elif visualNode.geometry.sphere.radius != 0:
            proto.write((shapeLevel + 1) * indent + 'geometry Sphere {\n')
            proto.write((shapeLevel + 2) * indent + 'radius ' + str(visualNode.geometry.sphere.radius) + '\n')
            proto.write((shapeLevel + 1) * indent + '}\n')

        elif visualNode.geometry.trimesh.coord != []:
            proto.write((shapeLevel + 1) * indent + 'geometry IndexedFaceSet {\n')
            proto.write((shapeLevel + 2) * indent + 'coord Coordinate {\n')
            proto.write((shapeLevel + 3) * indent + 'point [\n')
            for value in visualNode.geometry.trimesh.coord:
                proto.write(shapeLevel * indent + str(value[0] * visualNode.geometry.scale[0]) + ' ' + str(value[1] * visualNode.geometry.scale[1]) + ' ' + str(value[2] * visualNode.geometry.scale[2]) + '\n')
            proto.write((shapeLevel + 3) * indent + ']\n')
            proto.write((shapeLevel + 2) * indent + '}\n')

            proto.write((shapeLevel + 2) * indent + 'coordIndex [\n')
            for value in visualNode.geometry.trimesh.coordIndex:
                proto.write(shapeLevel * indent + str(value[0]) + ' ' + str(value[1]) + ' ' + str(value[2]) + ' -1\n')
            proto.write((shapeLevel + 2) * indent + ']\n')

            if visualNode.geometry.trimesh.texCoord != []:
                proto.write((shapeLevel + 2) * indent + 'texCoord TextureCoordinate {\n')
                proto.write((shapeLevel + 3) * indent + 'point [\n')
                for value in visualNode.geometry.trimesh.texCoord:
                    proto.write(shapeLevel * indent + str(value[0]) + ' ' + str(value[1]) + '\n')
                proto.write((shapeLevel + 3) * indent + ']\n')
                proto.write((shapeLevel + 2) * indent + '}\n')

                proto.write((shapeLevel + 2) * indent + 'texCoordIndex [\n')
                for value in visualNode.geometry.trimesh.texCoordIndex:
                    proto.write(shapeLevel * indent + str(value[0]) + ' ' + str(value[1]) + ' ' + str(value[2]) + ' -1\n')
                proto.write((shapeLevel + 2) * indent + ']\n')

            proto.write((shapeLevel + 2) * indent + 'creaseAngle 1\n')
            proto.write((shapeLevel + 1) * indent + '}\n')

        proto.write(shapeLevel * indent + '}\n')
        if transform:
            proto.write((shapeLevel - 1) * indent + ']\n')
            proto.write((shapeLevel - 2) * indent + '}\n')
            shapeLevel = shapeLevel - 2
    if group:
        proto.write((shapeLevel - 1) * indent + ']\n')
        proto.write((shapeLevel - 2) * indent + '}\n')


def URDFJoint(proto, joint, level, parentList, childList, linkList, jointList, boxCollision):
    indent = '  '
    if joint.type == 'revolute' or joint.type == 'continuous':
        proto.write(level * indent + ' HingeJoint {\n')
        proto.write((level + 1) * indent + 'jointParameters HingeJointParameters {\n')
        proto.write((level + 2) * indent + 'axis ' + str(joint.axis[0]) + ' ' + str(joint.axis[1]) + ' ' + str(joint.axis[2]) + '\n')
        proto.write((level + 2) * indent + 'anchor ' + str(joint.position[0]) + ' ' + str(joint.position[1]) + ' ' + str(joint.position[2]) + '\n')
        proto.write((level + 2) * indent + 'dampingConstant ' + str(joint.dynamics.damping) + '\n')
        proto.write((level + 2) * indent + 'staticFriction ' + str(joint.dynamics.friction) + '\n')
        proto.write((level + 1) * indent + '}\n')
        proto.write((level + 1) * indent + 'device RotationalMotor {\n')
    elif joint.type == 'prismatic':
        proto.write(level * indent + ' SliderJoint {\n')
        proto.write((level + 1) * indent + 'jointParameters JointParameters {\n')
        proto.write((level + 2) * indent + 'axis ' + str(joint.axis[0]) + ' ' + str(joint.axis[1]) + ' ' + str(joint.axis[2]) + '\n')
        proto.write((level + 2) * indent + 'dampingConstant ' + str(joint.dynamics.damping) + '\n')
        proto.write((level + 2) * indent + 'staticFriction ' + str(joint.dynamics.friction) + '\n')
        proto.write((level + 1) * indent + '}\n')
        proto.write((level + 1) * indent + 'device LinearMotor {\n')
    elif joint.type == 'fixed':
        for childLink in linkList:
            if childLink.name == joint.child:
                URDFLink(proto, childLink, level, parentList, childList, linkList, jointList, joint.position, joint.rotation, boxCollision)
        return

    elif joint.type == 'floating' or joint.type == 'planar':
        print joint.type + ' is not a supported joint type in Webots'
        return

    proto.write((level + 2) * indent + 'name "' + joint.name + '"\n')
    if joint.limit.velocity != 0.0:
        proto.write((level + 2) * indent + 'maxVelocity ' + str(joint.limit.velocity) + '\n')
    if joint.limit.lower != 0.0:
        proto.write((level + 2) * indent + 'minPosition ' + str(joint.limit.lower) + '\n')
    if joint.limit.upper != 0.0:
        proto.write((level + 2) * indent + 'maxPosition ' + str(joint.limit.upper) + '\n')
    if joint.limit.effort != 0.0:
        if joint.type == 'prismatic':
            proto.write((level + 2) * indent + 'maxForce ' + str(joint.limit.effort) + '\n')
        else:
            proto.write((level + 2) * indent + 'maxTorque ' + str(joint.limit.effort) + '\n')
    proto.write((level + 1) * indent + '}\n')

    proto.write((level + 1) * indent + 'endPoint')
    for childLink in linkList:
        if childLink.name == joint.child:
            URDFLink(proto, childLink, level + 1, parentList, childList, linkList, jointList, joint.position, joint.rotation, boxCollision)
    proto.write(level * indent + '}\n')
