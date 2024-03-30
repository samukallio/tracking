from numpy import array, block, eye, zeros, trace
from numpy.linalg import inv, norm, det
from scipy.linalg import expm
from numpyx import vec3, vec4, qmull, qrotv, xmat, rmat3q, rmat4q
from math import sin, cos, radians

class QuaternionIntegrator:
    def __init__(self):
        self._q = vec4(1.0, 0.0, 0.0, 0.0)

    @property
    def matrix(self):
        return rmat4q(self._q)

    def step(self, dt, rot):
        self._q = qmull(self._q, qrotv(dt * rot))
