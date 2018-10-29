import os
import sys
import xml
import struct
import math
import numpy
try:
    from PIL import Image
except ImportError as e:
    if sys.platform == 'linux2':
        sys.stderr.write("PIL module not found, please install it with:\n")
        sys.stderr.write("apt-get install python-pip\n")
        sys.stderr.write("pip install pillow\n")
    raise e

from collada import *

# Note: Such kind of global variables should be avoided.
# To do so, parserURDF.py should be a class.
pathPrefixes = []
counter = 0


class Trimesh():
    def __init__(self):
        self.coord = []  # list of coordinate points
        self.coordIndex = []  # list of index of points
        self.texCoord = []  # list of coordinate points for texture
        self.texCoordIndex = []  # list of index for texture


class Inertia():
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [1.0, 0.0, 0.0, 0.0]
        self.mass = 1.0
        self.ixx = 1.0
        self.ixy = 0.0
        self.ixz = 0.0
        self.iyy = 1.0
        self.iyz = 0.0
        self.izz = 1.0


class Box():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Cylinder():
    def __init__(self):
        self.radius = 0.0
        self.length = 0.0


class Sphere():
    def __init__(self):
        self.radius = 0.0


class Geometry():
    def __init__(self):
        self.box = Box()
        self.cylinder = Cylinder()
        self.sphere = Sphere()
        self.trimesh = Trimesh()
        self.scale = [1.0, 1.0, 1.0]


class Color():
    def __init__(self):
        self.red = 0.0
        self.green = 0.0
        self.blue = 0.0
        self.alpha = 0.0


class Material():
    def __init__(self):
        self.color = Color()
        self.texture = ""


class Visual():
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [1.0, 0.0, 0.0, 0.0]
        self.geometry = Geometry()
        self.material = Material()


class Collision():
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [1.0, 0.0, 0.0, 0.0]
        self.geometry = Geometry()


class Calibration():
    def __init__(self):
        self.limit = 0.0
        self.rising = True


class Dynamics():
    def __init__(self):
        self.damping = 0.0
        self.friction = 0.0


class Limit():
    def __init__(self):
        self.lower = 0.0
        self.upper = 0.0
        self.effort = 0.0
        self.velocity = 0.0


class Safety():
    def __init__(self):
        self.lower = 0.0
        self.upper = 0.0
        self.kPosition = 0.0
        self.kVelocity = 0.0


class Link():
    def __init__(self):
        self.name = 'default'
        self.inertia = Inertia()
        self.visual = []
        self.collision = []


class Joint():
    def __init__(self):
        self.name = 'default'
        self.type = 'default'
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [1.0, 0.0, 0.0, 0.0]
        self.parent = 'default'
        self.child = 'default'
        self.axis = []
        self.calibration = Calibration()
        self.dynamics = Dynamics()
        self.limit = Limit()
        self.safety = Safety()


def vector_norm(data, axis=None, out=None):
    data = numpy.array(data, dtype=numpy.float64, copy=True)
    if out is None:
        if data.ndim == 1:
            return math.sqrt(numpy.dot(data, data))
        data *= data
        out = numpy.atleast_1d(numpy.sum(data, axis=axis))
        numpy.sqrt(out, out)
        return out
    else:
        data *= data
        numpy.sum(data, axis=axis, out=out)
        numpy.sqrt(out, out)


def vrml_to_q(v):
    """Convert euler-axes-angle (vrml) to quaternion."""
    q = [0.0, 0.0, 0.0, 0.0]
    l = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
    if (l > 0.0):
        q[0] = math.cos(v[3] / 2)
        l = math.sin(v[3] / 2) / math.sqrt(l)
        q[1] = v[0] * l
        q[2] = v[1] * l
        q[3] = v[2] * l
    else:
        q[0] = 1
        q[1] = 0
        q[2] = 0
        q[3] = 0
    return q


def q_to_vrml(q):
    """Convert quaternion to euler-axes-angle (vrml)."""
    v = [0.0, 0.0, 0.0, 0.0]
    v[3] = 2.0 * math.acos(q[0])
    if (v[3] < 0.0001):
        # if v[3] close to zero then direction of axis not important
        v[0] = 0.0
        v[1] = 1.0
        v[2] = 0.0
    else:
        # normalise axes
        n = math.sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        v[0] = q[1] / n
        v[1] = q[2] / n
        v[2] = q[3] / n
    return v


def q_mult(qb, qc):
    """Quaternion multiplication (combining rotations)."""
    qa = [0.0, 0.0, 0.0, 0.0]
    qa[0] = qb[0] * qc[0] - qb[1] * qc[1] - qb[2] * qc[2] - qb[3] * qc[3]
    qa[1] = qb[0] * qc[1] + qb[1] * qc[0] + qb[2] * qc[3] - qb[3] * qc[2]
    qa[2] = qb[0] * qc[2] + qb[2] * qc[0] + qb[3] * qc[1] - qb[1] * qc[3]
    qa[3] = qb[0] * qc[3] + qb[3] * qc[0] + qb[1] * qc[2] - qb[2] * qc[1]
    return qa


def setPossiblePathPrefixes(prefixes):
    """Set possible path prefixes."""
    # Note: Path prefixes are used when files are defined relatively.
    pathPrefixes = prefixes


def convertRPYtoEulerAxis(rpy, cylinder=False):
    offset2 = 0.0
    if cylinder:
        offset2 = 1.57
    ea1 = [0.0, 1.0, 0.0, rpy[1]]
    ea2 = [1.0, 0.0, 0.0, rpy[0] + offset2]
    ea3 = [0.0, 0.0, 1.0, rpy[2]]
    # convert vrml to quaternion representation
    q1 = vrml_to_q(ea1)
    q2 = vrml_to_q(ea2)
    q3 = vrml_to_q(ea3)
    # combine 3 quaternions into 1
    qa = q_mult(q1, q2)
    qb = q_mult(qa, q3)
    # convert quaternion to vrml
    return q_to_vrml(qb)


def getRobotName(node):
    name = node.getAttribute('name')
    if name == 'drc_skeleton':
        name = 'training_atlas'
    print 'the name of the robot is ' + name
    return name


def getPlugins(node):
    pluginList = []
    for child in node.childNodes:
        if child.localName != 'link' and child.localName != 'joint' and child.nodeType == xml.dom.minidom.Node.ELEMENT_NODE:
                pluginList.append(child)
    return pluginList


def hasElement(node, element):
    if node.getElementsByTagName(element).length > 0:
        return True
    else:
        return False


def getSTLMesh(filename, node):
    stlFile = None
    if os.path.isfile(filename):
        stlFile = open(filename, 'rb')
    else:
        for path in pathPrefixes:
            filePath = os.path.join(path, filename)
            if os.path.isfile(filePath):
                stlFile = open(filePath, 'rb')
                break
        return

    stlFile.read(80)
    vertex1 = []
    vertex2 = []
    vertex3 = []
    numTriangles = struct.unpack("@i", stlFile.read(4))[0]
    struct.unpack("<3f", stlFile.read(12))
    a = struct.unpack("<3f", stlFile.read(12))
    vertex1.append(a)
    b = struct.unpack("<3f", stlFile.read(12))
    vertex2.append(b)
    c = struct.unpack("<3f", stlFile.read(12))
    vertex3.append(c)
    struct.unpack("H", stlFile.read(2))
    node.geometry.trimesh.coord.append(vertex1[0])
    node.geometry.trimesh.coord.append(vertex2[0])
    node.geometry.trimesh.coord.append(vertex3[0])
    for i in range(1, numTriangles):
        struct.unpack("<3f", stlFile.read(12))
        a = struct.unpack("<3f", stlFile.read(12))
        vertex1.append(a)
        if node.geometry.trimesh.coord.count(a) == 0:
            node.geometry.trimesh.coord.append(a)
        b = struct.unpack("<3f", stlFile.read(12))
        vertex2.append(b)
        if node.geometry.trimesh.coord.count(b) == 0:
            node.geometry.trimesh.coord.append(b)
        c = struct.unpack("<3f", stlFile.read(12))
        vertex3.append(c)
        if node.geometry.trimesh.coord.count(c) == 0:
            node.geometry.trimesh.coord.append(c)
        struct.unpack("H", stlFile.read(2))
        node.geometry.trimesh.coordIndex.append([node.geometry.trimesh.coord.index(vertex1[i]), node.geometry.trimesh.coord.index(vertex2[i]), node.geometry.trimesh.coord.index(vertex3[i])])
    stlFile.close()
    return node


def getColladaMesh(filename, node, link):
    colladaMesh = Collada(filename)
    if node.material:
        for geometry in list(colladaMesh.scene.objects('geometry')):
            visual = Visual()
            visual.position = node.position
            visual.rotation = node.rotation
            visual.material.color = node.material.color
            visual.material.texture = ""
            visual.geometry.scale = node.geometry.scale
            for value in list(geometry.primitives())[0].vertex:
                visual.geometry.trimesh.coord.append(numpy.array(value))
            for value in list(geometry.primitives())[0].vertex_index:
                visual.geometry.trimesh.coordIndex.append(value)
            for value in list(geometry.primitives())[0].texcoordset[0]:
                visual.geometry.trimesh.texCoord.append(value)
            for value in list(geometry.primitives())[0].texcoord_indexset[0]:
                visual.geometry.trimesh.texCoordIndex.append(value)
            if list(geometry.primitives())[0].material and list(geometry.primitives())[0].material.effect:
                visual.material.texture = 'textures/' + list(geometry.primitives())[0].material.effect.diffuse.sampler.surface.image.path.split('/')[-1]
                if os.path.splitext(visual.material.texture)[1] == '.tiff' or os.path.splitext(visual.material.texture)[1] == '.tif':
                    for dirname, dirnames, filenames in os.walk('.'):
                        for filename in filenames:
                            if filename == str(visual.material.texture.split('/')[-1]):
                                try:
                                    tifImage = Image.open(os.path.join(dirname, filename))
                                    tifImage.save(os.path.splitext(os.path.join(dirname, filename))[0] + '.png')
                                    visual.material.texture = 'textures/' + os.path.splitext(filename)[0] + '.png'
                                    print 'translated image', visual.material.texture
                                except:
                                    visual.material.texture = ""
                                    print 'failed to open ' + os.path.join(dirname, filename)
            link.visual.append(visual)
    else:
        for geometry in list(colladaMesh.scene.objects('geometry')):
            collision = Collision()
            collision.position = node.position
            collision.rotation = node.rotation
            collision.geometry.scale = node.geometry.scale
            for value in list(geometry.primitives())[0].vertex:
                collision.geometry.trimesh.coord.append(numpy.array(value))
            for value in list(geometry.primitives())[0].vertex_index:
                collision.geometry.trimesh.coordIndex.append(value)
            link.collision.append(collision)


# the use of collada.scene.object class makes this function useless for now but it may serves in the future
'''
def getColladaTransform(node, id):
    translation = [0.0, 0.0, 0.0]
    rotation = [1.0, 0.0, 0.0, 0.0]
    scale = [1.0, 1.0, 1.0]
    if node.children:
        for child in node.children:
            if str(child).count('geometry'):
                if node.transforms:
                    for transform in node.transforms:
                        if str(transform).count('Matrix'):
                            matrix = numpy.array(transform.matrix, dtype=numpy.float64, copy=True).T
                            row = matrix[:4, :4].copy()
                            matrixScale = vector_norm(row[0])
                            row[0] /= matrixScale
                            row[1] /= matrixScale
                            row[2] /= matrixScale
                            row[3] /= matrixScale #not sure about this one
                            translation[0] = matrix[3,0]
                            translation[1] = matrix[3,1]
                            translation[2] = matrix[3,2]
                            if row[0][0] != 1 or row[1][1] != 1 or row[2][2] != 1:
                                rotation[3] = math.acos((row[0][0] + row[1][1] + row[2][2] - 1.0) / 2.0)
                                rotation[0] = -(row[2][1] - row[1][2]) / 2.0 / math.sin(rotation[3])
                                rotation[1] = -(row[0][2] - row[2][0]) / 2.0 / math.sin(rotation[3])
                                rotation[2] = -(row[1][0] - row[0][1]) / 2.0 / math.sin(rotation[3])
                        elif str(transform).count('Translate'):
                            translation[0] = transform.x
                            translation[1] = transform.y
                            translation[2] = transform.z
                        elif str(transform).count('Rotate'):
                            rotation[0] = transform.x
                            rotation[1] = transform.y
                            rotation[2] = transform.z
                            rotation[3] = transform.angle
                        elif str(transform).count('Scale'):
                            scale[0] = transform.x
                            scale[1] = transform.y
                            scale[2] = transform.z

    if translation == [0.0, 0.0, 0.0] and rotation == [1.0, 0.0, 0.0, 0.0] and scale == [1.0, 1.0, 1.0]:
        if node.children:
            for node in node.children:
                if str(child).count('children'):
                    translation, rotation, scale = getColladaTransform(child, id)
    return translation, rotation, scale
'''


def getPosition(node):
    position = [0.0, 0.0, 0.0]
    positionString = node.getElementsByTagName('origin')[0].getAttribute('xyz').split()
    position[0] = float(positionString[0])
    position[1] = float(positionString[1])
    position[2] = float(positionString[2])
    return position


def getRotation(node, isCylinder=False):
    rotation = [0.0, 0.0, 0.0]
    if hasElement(node, 'origin'):
        orientationString = node.getElementsByTagName('origin')[0].getAttribute('rpy').split()
        rotation[0] = float(orientationString[0])
        rotation[1] = float(orientationString[1])
        rotation[2] = float(orientationString[2])
    if isCylinder:
        return convertRPYtoEulerAxis(rotation, True)
    else:
        return convertRPYtoEulerAxis(rotation, False)


def getInertia(node):
    inertia = Inertia()
    inertialElement = node.getElementsByTagName('inertial')[0]
    if hasElement(inertialElement, 'origin'):
        if inertialElement.getElementsByTagName('origin')[0].getAttribute('xyz'):
            inertia.position = getPosition(inertialElement)
        if inertialElement.getElementsByTagName('origin')[0].getAttribute('rpy'):
            inertia.rotation = getRotation(inertialElement)
    if hasElement(inertialElement, 'mass'):
        inertia.mass = float(inertialElement.getElementsByTagName('mass')[0].getAttribute('value'))
    if hasElement(inertialElement, 'inertia'):
        matrixNode = inertialElement.getElementsByTagName('inertia')[0]
        inertia.ixx = float(matrixNode.getAttribute('ixx'))
        inertia.ixy = float(matrixNode.getAttribute('ixy'))
        inertia.ixz = float(matrixNode.getAttribute('ixz'))
        inertia.iyy = float(matrixNode.getAttribute('iyy'))
        inertia.iyz = float(matrixNode.getAttribute('iyz'))
        inertia.izz = float(matrixNode.getAttribute('izz'))
    return inertia


def getVisual(link, node):
    for index in range(0, len(node.getElementsByTagName('visual'))):
        visual = Visual()
        visualElement = node.getElementsByTagName('visual')[index]
        if hasElement(visualElement, 'origin'):
            if visualElement.getElementsByTagName('origin')[0].getAttribute('xyz'):
                visual.position = getPosition(visualElement)
            if visualElement.getElementsByTagName('origin')[0].getAttribute('rpy'):
                if hasElement(visualElement.getElementsByTagName('geometry')[0], 'cylinder'):
                    visual.rotation = getRotation(visualElement, True)
                else:
                    visual.rotation = getRotation(visualElement)
        elif hasElement(visualElement.getElementsByTagName('geometry')[0], 'cylinder'):
                visual.rotation = getRotation(visualElement, True)

        geometryElement = visualElement.getElementsByTagName('geometry')[0]

        if hasElement(visualElement, 'material'):
            material = visualElement.getElementsByTagName('material')[0]

            # find the material reference if any
            if not material.hasChildNodes() and material.getAttribute('name'):
                node = material
                while node is not None and node.tagName != 'robot':
                    node = node.parentNode
                if node:
                    for child in node.childNodes:
                        if child.nodeType == xml.dom.minidom.Node.ELEMENT_NODE and child.tagName == 'material' and child.getAttribute('name') == material.getAttribute('name'):
                            material = child
                            break

            if hasElement(material, 'color'):
                colorElement = material.getElementsByTagName('color')[0].getAttribute('rgba').split()
                visual.material.color.red = float(colorElement[0])
                visual.material.color.green = float(colorElement[1])
                visual.material.color.blue = float(colorElement[2])
                visual.material.color.alpha = float(colorElement[3])
            if hasElement(material, 'texture'):
                visual.material.texture = material.getElementsByTagName('texture')[0].getAttribute('filename')
                if os.path.splitext(visual.material.texture)[1] == '.tiff' or os.path.splitext(visual.material.texture)[1] == '.tif':
                    for dirname, dirnames, filenames in os.walk('.'):
                        for filename in filenames:
                            if filename == str(visual.material.texture.split('/')[-1]):
                                print 'try to translate image', filename
                                try:
                                    tifImage = Image.open(os.path.join(dirname, filename))
                                    tifImage.save(os.path.splitext(os.path.join(dirname, filename))[0] + '.png')
                                    visual.material.texture = 'textures/' + os.path.splitext(filename)[0] + '.png'
                                except:
                                    visual.material.texture = ""
                                    print 'failed to open ' + os.path.join(dirname, filename)

        if hasElement(geometryElement, 'box'):
            visual.geometry.box.x = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[0])
            visual.geometry.box.y = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[1])
            visual.geometry.box.z = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[2])
            link.visual.append(visual)
        elif hasElement(geometryElement, 'cylinder'):
            visual.geometry.cylinder.radius = float(geometryElement.getElementsByTagName('cylinder')[0].getAttribute('radius'))
            visual.geometry.cylinder.length = float(geometryElement.getElementsByTagName('cylinder')[0].getAttribute('length'))
            link.visual.append(visual)
        elif hasElement(geometryElement, 'sphere'):
            visual.geometry.sphere.radius = float(geometryElement.getElementsByTagName('sphere')[0].getAttribute('radius'))
            link.visual.append(visual)
        elif hasElement(geometryElement, 'mesh'):
            meshfile = geometryElement.getElementsByTagName('mesh')[0].getAttribute('filename')
            # hack for gazebo mesh database
            if meshfile.count('package'):
                meshfile = str(meshfile.split('/')[-2]) + '/' + str(meshfile.split('/')[-1])
            if geometryElement.getElementsByTagName('mesh')[0].getAttribute('scale'):
                meshScale = geometryElement.getElementsByTagName('mesh')[0].getAttribute('scale').split()
                visual.geometry.scale[0] = float(meshScale[0])
                visual.geometry.scale[1] = float(meshScale[1])
                visual.geometry.scale[2] = float(meshScale[2])
            if os.path.splitext(meshfile)[1].lower() == '.dae':
                getColladaMesh(meshfile, visual, link)
            elif os.path.splitext(meshfile)[1].lower() == '.stl':
                visual = getSTLMesh(meshfile, visual)
                link.visual.append(visual)


def getCollision(link, node):
    for index in range(0, len(node.getElementsByTagName('collision'))):
        collision = Collision()
        collisionElement = node.getElementsByTagName('collision')[index]
        if hasElement(collisionElement, 'origin'):
            if collisionElement.getElementsByTagName('origin')[0].getAttribute('xyz'):
                collision.position = getPosition(collisionElement)
            if collisionElement.getElementsByTagName('origin')[0].getAttribute('rpy'):
                if hasElement(collisionElement.getElementsByTagName('geometry')[0], 'cylinder'):
                    collision.rotation = getRotation(collisionElement, True)
                else:
                    collision.rotation = getRotation(collisionElement)
        elif hasElement(collisionElement.getElementsByTagName('geometry')[0], 'cylinder'):
                collision.rotation = getRotation(collisionElement, True)

        geometryElement = collisionElement.getElementsByTagName('geometry')[0]
        if hasElement(geometryElement, 'box'):
            collision.geometry.box.x = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[0])
            collision.geometry.box.y = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[1])
            collision.geometry.box.z = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[2])
            link.collision.append(collision)
        elif hasElement(geometryElement, 'cylinder'):
            collision.geometry.cylinder.radius = float(geometryElement.getElementsByTagName('cylinder')[0].getAttribute('radius'))
            collision.geometry.cylinder.length = float(geometryElement.getElementsByTagName('cylinder')[0].getAttribute('length'))
            link.collision.append(collision)
        elif hasElement(geometryElement, 'sphere'):
            collision.geometry.sphere.radius = float(geometryElement.getElementsByTagName('sphere')[0].getAttribute('radius'))
            link.collision.append(collision)
        elif hasElement(geometryElement, 'mesh'):
            meshfile = geometryElement.getElementsByTagName('mesh')[0].getAttribute('filename')
            if geometryElement.getElementsByTagName('mesh')[0].getAttribute('scale'):
                meshScale = geometryElement.getElementsByTagName('mesh')[0].getAttribute('scale').split()
                collision.geometry.scale[0] = float(meshScale[0])
                collision.geometry.scale[1] = float(meshScale[1])
                collision.geometry.scale[2] = float(meshScale[2])
            # hack for gazebo mesh database
            if meshfile.count('package'):
                meshfile = str(meshfile.split('/')[-2]) + '/' + str(meshfile.split('/')[-1])
            if os.path.splitext(meshfile)[1] == '.dae':
                collision.geometry.collada = getColladaMesh(meshfile, collision, link)
            elif os.path.splitext(meshfile)[1] == '.stl':
                collision.geometry.stl = getSTLMesh(meshfile, collision)
                link.collision.append(collision)


def getAxis(node):
    axis = [0.0, 0.0, 0.0]
    axisElement = node.getElementsByTagName('axis')[0].getAttribute('xyz').split()
    axis[0] = float(axisElement[0])
    axis[1] = float(axisElement[1])
    axis[2] = float(axisElement[2])
    return axis


def getCalibration(node):
    calibration = Calibration()
    calibrationElement = node.getElementsByTagName('calibration')[0]
    if hasElement(calibrationElement, 'rising'):
        calibration.limit = calibrationElement.getAttribute('rising')
        calibration.rising = True
    else:
        calibration.limit = calibrationElement.getAttribute('falling')
        calibration.rising = False
    return calibration


def getDynamics(node):
    dynamics = Dynamics()
    dynamicsElement = node.getElementsByTagName('dynamics')[0]
    if dynamicsElement.getAttribute('damping'):
        dynamics.damping = float(dynamicsElement.getAttribute('damping'))
    if dynamicsElement.getAttribute('friction'):
        dynamics.friction = float(dynamicsElement.getAttribute('friction'))
    return dynamics


def getLimit(node):
    limit = Limit()
    limitElement = node.getElementsByTagName('limit')[0]
    if limitElement.getAttribute('lower'):
        limit.lower = float(limitElement.getAttribute('lower'))
    if limitElement.getAttribute('upper'):
        limit.upper = float(limitElement.getAttribute('upper'))
    limit.effort = float(limitElement.getAttribute('effort'))
    limit.velocity = float(limitElement.getAttribute('velocity'))
    return limit


def getSafety(node):
    safety = Safety()
    if node.getElementsByTagName('safety_controller')[0].getAttribute('soft_lower_limit'):
        safety.lower = float(node.getElementsByTagName('safety_controller')[0].getAttribute('soft_lower_limit'))
    if node.getElementsByTagName('safety_controller')[0].getAttribute('soft_upper_limit'):
        safety.upper = float(node.getElementsByTagName('safety_controller')[0].getAttribute('soft_upper_limit'))
    if node.getElementsByTagName('safety_controller')[0].getAttribute('k_position'):
        safety.kPosition = float(node.getElementsByTagName('safety_controller')[0].getAttribute('k_position'))
    safety.kVelocity = float(node.getElementsByTagName('safety_controller')[0].getAttribute('k_velocity'))
    return safety


def getLink(node):
    link = Link()
    link.name = node.getAttribute('name')
    if hasElement(node, 'inertial'):
        link.inertia = getInertia(node)
    if hasElement(node, 'visual'):
        getVisual(link, node)
    if hasElement(node, 'collision'):
        getCollision(link, node)
    return link


def getJoint(node):
    joint = Joint()
    joint.name = node.getAttribute('name')
    joint.type = node.getAttribute('type')
    if hasElement(node, 'origin'):
        if node.getElementsByTagName('origin')[0].getAttribute('xyz'):
            joint.position = getPosition(node)
        if node.getElementsByTagName('origin')[0].getAttribute('rpy'):
            joint.rotation = getRotation(node)
    joint.parent = node.getElementsByTagName('parent')[0].getAttribute('link')
    joint.child = node.getElementsByTagName('child')[0].getAttribute('link')
    if hasElement(node, 'axis'):
        joint.axis = getAxis(node)
    if hasElement(node, 'calibration'):
        joint.calibration = getCalibration(node)
    if hasElement(node, 'dynamics'):
        joint.dynamics = getDynamics(node)
    if hasElement(node, 'limit'):
        joint.limit = getLimit(node)
    if hasElement(node, 'safety_controller'):
        joint.safety = getSafety(node)
    return joint


def isRootLink(link, childList):
    for child in childList:
        if link == child:
            return False
    return True
