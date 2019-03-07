"""Import modules."""
import numpy as np


class RGB():
    """RGB color object."""

    def __init__(self):
        """Initialization."""
        self.red = 0.0
        self.green = 0.0
        self.blue = 0.0


# ref: https://marcodiiga.github.io/rgba-to-rgb-conversion
def RGBA2RGB(RGBA_color, RGB_background=RGB()):
    """Convert RGBA to RGB expression."""
    alpha = RGBA_color.alpha

    new_color = RGB()
    new_color.red = (1 - alpha) * RGB_background.red + alpha * RGBA_color.red
    new_color.green = (1 - alpha) * RGB_background.green + alpha * RGBA_color.green
    new_color.blue = (1 - alpha) * RGB_background.blue + alpha * RGBA_color.blue

    return new_color


def header(proto, srcFile, robotName):
    """Specify VRML file header."""
    proto.write('#VRML_SIM R2019a utf8\n')
    proto.write('# license: Apache License 2.0\n')
    proto.write('# license url: http://www.apache.org/licenses/LICENSE-2.0\n')
    proto.write('# This is a proto file for Webots for the ' + robotName + '\n')
    proto.write('# Extracted from: ' + srcFile + '\n\n')


def declaration(proto, robotName):
    """Prototype declaration."""
    proto.write('PROTO ' + robotName + ' [\n')
    proto.write('   field  SFVec3f     translation  0 0 0\n')
    proto.write('   field  SFRotation  rotation     0 1 0 0\n')
    proto.write('   field  SFString    controller   "void"\n')
    proto.write(']\n')
    proto.write('{\n')


def URDFLink(proto, link, level, parentList, childList, linkList, jointList,
             jointPosition=[0.0, 0.0, 0.0], jointRotation=[1.0, 0.0, 0.0, 0.0],
             boxCollision=False, dummy=False, robot=False, endpoint=False):
    """Write a link iteratively."""
    indent = '  '
    haveChild = False
    if robot:
        proto.write(level * indent + 'Robot {\n')
        proto.write((level + 1) * indent + 'translation IS translation\n')
        proto.write((level + 1) * indent + 'rotation IS rotation\n')
        proto.write((level + 1) * indent + 'controller IS controller\n')
    else:
        proto.write((' ' if endpoint else level * indent) + 'Solid {\n')
        proto.write((level + 1) * indent + 'translation %lf %lf %lf\n' % (jointPosition[0], jointPosition[1], jointPosition[2]))
        proto.write((level + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (jointRotation[0], jointRotation[1], jointRotation[2], jointRotation[3]))
    if dummy:  # case when link not defined but referenced (e.g. Atlas robot)
        pass
    else:
        if link.visual:
            proto.write((level + 1) * indent + 'children [\n')
            haveChild = True
            URDFShape(proto, link, level + 2)

        for joint in jointList:
            if joint.parent == link.name:
                if not haveChild:
                    haveChild = True
                    proto.write((level + 1) * indent + 'children [\n')
                URDFJoint(proto, joint, level + 2, parentList, childList,
                          linkList, jointList, boxCollision)
        if haveChild:
            proto.write((level + 1) * indent + ']\n')

        proto.write((level + 1) * indent + 'name "' + link.name + '"\n')

        if link.collision:
            URDFBoundingObject(proto, link, level + 1, boxCollision)

        proto.write((level + 1) * indent + 'physics Physics {\n')
        proto.write((level + 2) * indent + 'density -1\n')
        proto.write((level + 2) * indent + 'mass %lf\n' % link.inertia.mass)
        proto.write((level + 2) * indent + 'inertiaMatrix [ %lf %lf %lf, %lf %lf %lf ]\n' % (link.inertia.ixx, link.inertia.iyy, link.inertia.izz, link.inertia.ixy, link.inertia.ixz, link.inertia.iyz))
        proto.write((level + 2) * indent + 'centerOfMass [ %lf %lf %lf ]\n' % (link.inertia.position[0], link.inertia.position[1], link.inertia.position[2]))
        proto.write((level + 1) * indent + '}\n')

        if link.inertia.rotation[-1] != 0.0:  # this should not happend
            print('Warning: inertia of %s has a non-zero rotation [axis-angle] = "%lf %lf %lf %lf" but it will not be imported in proto!' % (link.name, link.inertia.rotation[0], link.inertia.rotation[1], link.inertia.rotation[2], link.inertia.rotation[3]))

    proto.write(level * indent + '}\n')


def URDFBoundingObject(proto, link, level, boxCollision):
    """Write an boundingObject."""
    indent = '  '
    boundingLevel = level
    proto.write(level * indent + 'boundingObject ')
    hasGroup = len(link.collision) > 1
    if hasGroup:
        proto.write('Group {\n')
        proto.write((level + 1) * indent + 'children [\n')
        boundingLevel = level + 2

    for boundingObject in link.collision:
        initialIndent = boundingLevel * indent if hasGroup else ''
        if boundingObject.position != [0.0, 0.0, 0.0] or boundingObject.rotation[3] != 0.0:
            proto.write(initialIndent + 'Transform {\n')
            proto.write((boundingLevel + 1) * indent + 'translation %lf %lf %lf\n' % (boundingObject.position[0], boundingObject.position[1], boundingObject.position[2]))
            proto.write((boundingLevel + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (boundingObject.rotation[0], boundingObject.rotation[1], boundingObject.rotation[2], boundingObject.rotation[3]))
            proto.write((boundingLevel + 1) * indent + 'children [\n')
            boundingLevel = boundingLevel + 2
            hasGroup = True
            initialIndent = boundingLevel * indent

        if boundingObject.geometry.box.x != 0:
            proto.write(initialIndent + 'Box {\n')
            proto.write((boundingLevel + 1) * indent + ' size %lf %lf %lf\n' % (boundingObject.geometry.box.x, boundingObject.geometry.box.y, boundingObject.geometry.box.z))
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.cylinder.radius != 0 and boundingObject.geometry.cylinder.length != 0:
            proto.write(initialIndent + 'Cylinder {\n')
            proto.write((boundingLevel + 1) * indent + 'radius ' + str(boundingObject.geometry.cylinder.radius) + '\n')
            proto.write((boundingLevel + 1) * indent + 'height ' + str(boundingObject.geometry.cylinder.length) + '\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.sphere.radius != 0:
            proto.write(initialIndent + 'Sphere {\n')
            proto.write((boundingLevel + 1) * indent + 'radius ' + str(boundingObject.geometry.sphere.radius) + '\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.trimesh.coord != [] and boxCollision:
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

            proto.write(initialIndent + 'Transform {\n')
            proto.write((boundingLevel + 2) * indent + 'translation %f %f %f\n' % (
                        0.5 * (aabb['maximum']['x'] + aabb['minimum']['x']),
                        0.5 * (aabb['maximum']['y'] + aabb['minimum']['y']),
                        0.5 * (aabb['maximum']['z'] + aabb['minimum']['z']),))
            proto.write((boundingLevel + 1) * indent + 'children [\n')
            proto.write((boundingLevel + 2) * indent + 'Box {\n')
            proto.write((boundingLevel + 3) * indent + 'size %f %f %f\n' % (
                        aabb['maximum']['x'] - aabb['minimum']['x'],
                        aabb['maximum']['y'] - aabb['minimum']['y'],
                        aabb['maximum']['z'] - aabb['minimum']['z'],))
            proto.write((boundingLevel + 2) * indent + '}\n')
            proto.write((boundingLevel + 1) * indent + ']\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.trimesh.coord != []:
            proto.write(initialIndent + 'IndexedFaceSet {\n')

            proto.write((boundingLevel + 1) * indent + 'coord Coordinate {\n')
            proto.write((boundingLevel + 2) * indent + 'point [\n' + (boundingLevel + 3) * indent)
            for value in boundingObject.geometry.trimesh.coord:
                proto.write('%lf %lf %lf, ' % (value[0] * boundingObject.geometry.scale[0], value[1] * boundingObject.geometry.scale[1], value[2] * boundingObject.geometry.scale[2]))
            proto.write('\n' + (boundingLevel + 2) * indent + ']\n')
            proto.write((boundingLevel + 1) * indent + '}\n')

            proto.write((boundingLevel + 1) * indent + 'coordIndex [\n' + (boundingLevel + 2) * indent)
            for value in boundingObject.geometry.trimesh.coordIndex:
                proto.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
            proto.write('\n' + (boundingLevel + 1) * indent + ']\n')
            proto.write(boundingLevel * indent + '}\n')

        else:
            proto.write(initialIndent + 'Box{\n')
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
    """Write a Shape."""
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
        if visualNode.position != [0.0, 0.0, 0.0] or visualNode.rotation[3] != 0:
            proto.write(shapeLevel * indent + 'Transform {\n')
            proto.write((shapeLevel + 1) * indent + 'translation ' +
                        str(visualNode.position[0]) + ' ' +
                        str(visualNode.position[1]) + ' ' +
                        str(visualNode.position[2]) + '\n')
            proto.write((shapeLevel + 1) * indent + 'rotation ' +
                        str(visualNode.rotation[0]) + ' ' +
                        str(visualNode.rotation[1]) + ' ' +
                        str(visualNode.rotation[2]) + ' ' +
                        str(visualNode.rotation[3]) + '\n')
            proto.write((shapeLevel + 1) * indent + 'children [\n')
            shapeLevel = shapeLevel + 2
            transform = True

        proto.write(shapeLevel * indent + 'Shape {\n')
        proto.write((shapeLevel + 1) * indent + 'appearance PBRAppearance {\n')
        ambientColor = RGBA2RGB(visualNode.material.ambient)
        diffuseColor = RGBA2RGB(visualNode.material.diffuse, RGB_background=ambientColor)
        emissiveColor = RGBA2RGB(visualNode.material.emission, RGB_background=ambientColor)
        roughness = 1.0 - visualNode.material.specular.alpha * (visualNode.material.specular.red + visualNode.material.specular.green + visualNode.material.specular.blue) / 3.0
        if visualNode.material.shininess:
            roughness *= (1.0 - 0.5 * visualNode.material.shininess)
        proto.write((shapeLevel + 2) * indent + 'baseColor %lf %lf %lf\n' % (diffuseColor.red, diffuseColor.green, diffuseColor.blue))
        proto.write((shapeLevel + 2) * indent + 'transparency %lf\n' % (1.0 - visualNode.material.diffuse.alpha))
        proto.write((shapeLevel + 2) * indent + 'roughness %lf\n' % roughness)
        proto.write((shapeLevel + 2) * indent + 'metalness 0\n')
        proto.write((shapeLevel + 2) * indent + 'emissiveColor %lf %lf %lf\n' % (emissiveColor.red, emissiveColor.green, emissiveColor.blue))
        if visualNode.material.texture != "":
            proto.write((shapeLevel + 2) * indent + 'baseColorMap ImageTexture {\n')
            proto.write((shapeLevel + 3) * indent + 'url [ "' + visualNode.material.texture + '" ]\n')
            proto.write((shapeLevel + 2) * indent + '}\n')
        proto.write((shapeLevel + 1) * indent + '}\n')

        if visualNode.geometry.box.x != 0:
            proto.write((shapeLevel + 1) * indent + 'geometry Box {\n')
            proto.write((shapeLevel + 2) * indent + ' size ' +
                        str(visualNode.geometry.box.x) + ' ' +
                        str(visualNode.geometry.box.y) + ' ' +
                        str(visualNode.geometry.box.z) + '\n')
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
            proto.write((shapeLevel + 3) * indent + 'point [\n' + (shapeLevel + 4) * indent)
            for value in visualNode.geometry.trimesh.coord:
                proto.write('%lf %lf %lf, ' % (value[0] * visualNode.geometry.scale[0], value[1] * visualNode.geometry.scale[1], value[2] * visualNode.geometry.scale[2]))
            proto.write('\n' + (shapeLevel + 3) * indent + ']\n')
            proto.write((shapeLevel + 2) * indent + '}\n')

            proto.write((shapeLevel + 2) * indent + 'coordIndex [\n' + (shapeLevel + 3) * indent)
            for value in visualNode.geometry.trimesh.coordIndex:
                proto.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
            proto.write('\n' + (shapeLevel + 2) * indent + ']\n')

            if visualNode.geometry.trimesh.texCoord != []:
                proto.write((shapeLevel + 2) * indent + 'texCoord TextureCoordinate {\n')
                proto.write((shapeLevel + 3) * indent + 'point [\n' + (shapeLevel + 4) * indent)
                for value in visualNode.geometry.trimesh.texCoord:
                    proto.write('%lf %lf, ' % (value[0], value[1]))
                proto.write('\n' + (shapeLevel + 3) * indent + ']\n')
                proto.write((shapeLevel + 2) * indent + '}\n')

                proto.write((shapeLevel + 2) * indent + 'texCoordIndex [\n' + (shapeLevel + 3) * indent)
                for value in visualNode.geometry.trimesh.texCoordIndex:
                    proto.write("%d %d %d -1 " % (value[0], value[1], value[2]))
                proto.write('\n' + (shapeLevel + 2) * indent + ']\n')

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


def URDFJoint(proto, joint, level, parentList, childList, linkList, jointList,
              boxCollision):
    """Write a Joint iteratively."""
    indent = '  '
    if joint.type == 'revolute' or joint.type == 'continuous':
        proto.write(level * indent + 'HingeJoint {\n')
        proto.write((level + 1) * indent + 'jointParameters HingeJointParameters {\n')
        proto.write((level + 2) * indent + 'axis ' +
                    str(joint.axis[0]) + ' ' +
                    str(joint.axis[1]) + ' ' +
                    str(joint.axis[2]) + '\n')
        proto.write((level + 2) * indent + 'anchor ' +
                    str(joint.position[0]) + ' ' +
                    str(joint.position[1]) + ' ' +
                    str(joint.position[2]) + '\n')
        proto.write((level + 2) * indent + 'dampingConstant ' + str(joint.dynamics.damping) + '\n')
        proto.write((level + 2) * indent + 'staticFriction ' + str(joint.dynamics.friction) + '\n')
        proto.write((level + 1) * indent + '}\n')
        proto.write((level + 1) * indent + 'device RotationalMotor {\n')
    elif joint.type == 'prismatic':
        proto.write(level * indent + 'SliderJoint {\n')
        proto.write((level + 1) * indent + 'jointParameters JointParameters {\n')
        proto.write((level + 2) * indent + 'axis ' +
                    str(joint.axis[0]) + ' ' +
                    str(joint.axis[1]) + ' ' +
                    str(joint.axis[2]) + '\n')
        proto.write((level + 2) * indent + 'dampingConstant ' + str(joint.dynamics.damping) + '\n')
        proto.write((level + 2) * indent + 'staticFriction ' + str(joint.dynamics.friction) + '\n')
        proto.write((level + 1) * indent + '}\n')
        proto.write((level + 1) * indent + 'device LinearMotor {\n')
    elif joint.type == 'fixed':
        for childLink in linkList:
            if childLink.name == joint.child:
                URDFLink(proto, childLink, level, parentList, childList,
                         linkList, jointList, joint.position, joint.rotation,
                         boxCollision)
        return

    elif joint.type == 'floating' or joint.type == 'planar':
        print(joint.type + ' is not a supported joint type in Webots')
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
    found_link = False
    for childLink in linkList:
        if childLink.name == joint.child:
            URDFLink(proto, childLink, level + 1, parentList, childList,
                     linkList, jointList, joint.position, joint.rotation,
                     boxCollision, endpoint=True)
            assert(not found_link)
            found_link = True
    # case that non-existing link cited, set dummy flag
    if not found_link and joint.child:
        URDFLink(proto, joint.child, level + 1, parentList, childList,
                 linkList, jointList, joint.position, joint.rotation,
                 boxCollision, dummy=True)
        print('warning: link ' + joint.child + ' is dummy!')
    proto.write(level * indent + '}\n')
