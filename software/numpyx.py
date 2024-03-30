from numpy import array, ndarray, zeros, eye, diag, dot, block, newaxis, float32, allclose, isclose
from numpy.random import normal
from numpy.linalg import inv
from math import sin, cos, sqrt

def vec1(x):
    """Make a (degenerate) 1-element column vector [x]."""
    return array(x).reshape(1, -1)

def vec3(x, y, z):
    """Make a 3-element column vector [x,y,z]^T."""
    return array([[x], [y], [z]])

def vec4(x, y, z, w):
    """Make a 4-element column vector [x,y,z,w]^T."""
    return array([[x], [y], [z], [w]])

def xmat(v):
    """Compute the left cross-product matrix of v."""
    return array([
        [0, -v[2][0], v[1][0]],
        [v[2][0], 0, -v[0][0]],
        [-v[1][0], v[0][0], 0]
    ])

def qmatl(q):
    """Compute the left (Hamiltonian) quaternion product matrix of q."""
    q0, qv = q[0:1], q[1:4]
    return block([
        [q0, -qv.T],
        [qv, q0*eye(3) + xmat(qv)]
    ])

def qmatr(q):
    """Compute the right quaternion product matrix of q."""
    q0, qv = q[0:1], q[1:4]
    return block([
        [q0, -qv.T],
        [qv, q0*eye(3) - xmat(qv)]
    ])

def qconj(q):
    """Compute the conjugate of quaternion q."""
    return array([
        q[0], -q[1], -q[2], -q[3]
    ])

def qinv(q):
    """Compute the inverse of quaternion q."""
    return qconj(q) / (q.T @ q)

def qmull(p, q):
    """Hamiltonian quaternion product of p and q."""
    return qmatl(p) @ q

def qmulr(p, q):
    return qmatr(p) @ q

def qrotv(v):
    d = sqrt(v.T @ v)
    if d < 10e-9:
        return vec4(1.0, 0.0, 0.0, 0.0)
    return array([
        [cos(d/2)],
        sin(d/2) * v[0] / d,
        sin(d/2) * v[1] / d,
        sin(d/2) * v[2] / d
    ])

def mat3to4(m):
    nv = vec3(0,0,0)
    return block([[m, nv], [nv.T, 1]])

def rmat3q(q):
    q0, qv = q[0:1], q[1:4]
    xv = xmat(qv)
    return qv @ qv.T + eye(3)*q[0]**2 + 2*q[0]*xv + xv @ xv

def rmat4q(q):
    q0, qv = q[0:1], q[1:4]
    xv = xmat(qv)
    return mat3to4(qv @ qv.T + eye(3)*q[0]**2 + 2*q[0]*xv + xv @ xv)

def rmatuv(u, v):
    u = u / sqrt(u.T @ u)
    v = v / sqrt(v.T @ v)
    w = xmat(u) @ v
    if not allclose(w, 0):
        s = w.T @ w
        c = u.T @ v
        wx = xmat(w)
        return mat3to4(eye(3) + wx + wx @ wx / (1 - c))
    else:
        if isclose(u.T @ v, 1):
            return eye(4)
        else:
            return mat3to4(-eye(3))

def tmatxyz(tx, ty, tz):
    return array([
            [1.0, 0.0, 0.0, tx],
            [0.0, 1.0, 0.0, ty],
            [0.0, 0.0, 1.0, tz],
            [0.0, 0.0, 0.0, 1.0]
        ], float32)

def smatxyz(sx, sy, sz):
    return array([
        [sx, 0.0, 0.0, 0.0],
        [0.0, sy, 0.0, 0.0],
        [0.0, 0.0, sz, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], float32)

def rmatxyz(rx, ry, rz):
    rmatx = array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, cos(rx), -sin(rx), 0.0],
        [0.0, sin(rx), cos(rx), 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], float32)

    rmaty = array([
        [cos(ry), 0.0, sin(ry), 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-sin(ry), 0.0, cos(ry), 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], float32)

    rmatz = array([
        [cos(rz), -sin(rz), 0.0, 0.0],
        [sin(rz), cos(rz), 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ], float32)

    return rmatz @ rmaty @ rmatx

def mat2wnd(x0, y0, x1, y1, sx=1.0, sy=1.0):
    return tmatxyz((x0+x1)/2,(y0+y1)/2,0) @ smatxyz(sx*(x1-x0)/2, sy*(y1-y0)/2, 0.0)
