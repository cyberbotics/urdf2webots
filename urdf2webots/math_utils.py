"""Import modules."""
import math
import numpy


def vectorNorm(data, axis=None, out=None):
    """Calculate norm of a vector."""
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


def rotationFromQuaternion(q):
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


def convertRPYtoQuaternions(rpy, cylinder=False):
    """Convert RPY to quaternions."""
    if cylinder:
        rpy[0] += 0.5 * math.pi
    cy = math.cos(rpy[2] * 0.5)
    sy = math.sin(rpy[2] * 0.5)
    cp = math.cos(rpy[1] * 0.5)
    sp = math.sin(rpy[1] * 0.5)
    cr = math.cos(rpy[0] * 0.5)
    sr = math.sin(rpy[0] * 0.5)

    q = [0, 0, 0, 0]
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr
    return q


def convertRPYtoEulerAxis(rpy, cylinder=False):
    """Convert RPY angles to Euler angles."""
    return rotationFromQuaternion(convertRPYtoQuaternions(rpy, cylinder))


def multiplyMatrix(mat1, mat2):
    """Multiply two matrices."""
    matrix = []
    matrix.append(mat1[0] * mat2[0] + mat1[1] * mat2[3] + mat1[2] * mat2[6])
    matrix.append(mat1[0] * mat2[1] + mat1[1] * mat2[4] + mat1[2] * mat2[7])
    matrix.append(mat1[0] * mat2[2] + mat1[1] * mat2[5] + mat1[2] * mat2[8])
    matrix.append(mat1[3] * mat2[0] + mat1[4] * mat2[3] + mat1[5] * mat2[6])
    matrix.append(mat1[3] * mat2[1] + mat1[4] * mat2[4] + mat1[5] * mat2[7])
    matrix.append(mat1[3] * mat2[2] + mat1[4] * mat2[5] + mat1[5] * mat2[8])
    matrix.append(mat1[6] * mat2[0] + mat1[7] * mat2[3] + mat1[8] * mat2[6])
    matrix.append(mat1[6] * mat2[1] + mat1[7] * mat2[4] + mat1[8] * mat2[7])
    matrix.append(mat1[6] * mat2[2] + mat1[7] * mat2[5] + mat1[8] * mat2[8])
    return matrix


def matrixFromRotation(rotation):
    """Get the 3x3 matrix associated to this VRML rotation."""
    c = math.cos(rotation[3])
    s = math.sin(rotation[3])
    t1 = 1.0 - c
    t2 = rotation[0] * rotation[2] * t1
    t3 = rotation[0] * rotation[1] * t1
    t4 = rotation[1] * rotation[2] * t1
    matrix = []
    matrix.append(rotation[0] * rotation[0] * t1 + c)
    matrix.append(t3 - rotation[2] * s)
    matrix.append(t2 + rotation[1] * s)
    matrix.append(t3 + rotation[2] * s)
    matrix.append(rotation[1] * rotation[1] * t1 + c)
    matrix.append(t4 - rotation[0] * s)
    matrix.append(t2 - rotation[1] * s)
    matrix.append(t4 + rotation[0] * s)
    matrix.append(rotation[2] * rotation[2] * t1 + c)
    return matrix


def rotationFromMatrix(R):
    R = numpy.array(R).reshape(3, 3)
    # code from here (slightly modified):
    # https://rock-learning.github.io/pytransform3d/_modules/pytransform3d/rotations.html#axis_angle_from_matrix
    angle = numpy.arccos((numpy.trace(R) - 1.0) / 2.0)
    epsilon = 1e-4
    if angle < epsilon:
        return numpy.array([1.0, 0.0, 0.0, 0.0])
    a = numpy.empty(4)

    # We can usually determine the rotation axis by inverting Rodrigues'
    # formula. Subtracting opposing off-diagonal elements gives us
    # 2 * sin(angle) * e,
    # where e is the normalized rotation axis.
    axis_unnormalized = numpy.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])

    if numpy.pi - angle < epsilon:
        # The threshold is a result from this discussion:
        # https://github.com/rock-learning/pytransform3d/issues/43
        # The standard formula becomes numerically unstable, however,
        # Rodrigues' formula reduces to R = I + 2 (ee^T - I), with the
        # rotation axis e, that is, ee^T = 0.5 * (R + I) and we can find the
        # squared values of the rotation axis on the diagonal of this matrix.
        # We can still use the original formula to reconstruct the signs of
        # the rotation axis correctly.
        sign = numpy.sign(axis_unnormalized)
        a[:3] = numpy.sqrt(0.5 * (numpy.diag(R) + 1.0)) * numpy.where(sign == 0, 1, sign)
        # print('test', abs(angle - numpy.pi), numpy.diag(R), numpy.sqrt(0.5 * (numpy.diag(R) + 1.0)), axis_unnormalized)
    else:
        a[:3] = axis_unnormalized
        # The norm of axis_unnormalized is 2.0 * numpy.sin(angle), that is, we
        # could normalize with a[:3] = a[:3] / (2.0 * numpy.sin(angle)),
        # but the following is much more precise for angles close to 0 or pi:
    a[:3] /= numpy.linalg.norm(a[:3])

    a[3] = angle
    return a


def rotateVector(vector, rotation):
    """Rotate the vector by the VRML rotation."""
    # multiply matrix by vector
    matrix = matrixFromRotation(rotation)
    v = []
    v.append(vector[0] * matrix[0] + vector[1] * matrix[1] + vector[2] * matrix[2])
    v.append(vector[0] * matrix[3] + vector[1] * matrix[4] + vector[2] * matrix[5])
    v.append(vector[0] * matrix[6] + vector[1] * matrix[7] + vector[2] * matrix[8])
    return v
