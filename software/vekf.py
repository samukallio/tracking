from numpy import array, block, eye
from numpy.linalg import inv
from scipy.linalg import expm
from numpyx import vec3, vec4, xmat
from math import sin, cos, radians

class VectorEKF:
    def __init__(self, r):
        self._r = r

        self._x = vec3(1, 0, 0)
        self._P = eye(3)
        self._var = 1.0

        self._matrix = eye(3)

    @property
    def covariance(self):
        return self._P

    @property
    def vector(self):
        return self._x

    def step(self, dt, vec, rot):
#        self._var = 0.9 * self._var + 0.1 * (self._r + 4.0 * abs(1.0 - vec.T @ vec) ** 2)
        self._var = self._r

        R = eye(3) * self._var
        rrot = 0.01

        # Prediction step
        F = expm(-dt * xmat(rot))
        xp = F @ self._x
        Pp = F @ self._P @ F.T - rrot * dt * xmat(self._x) @ xmat(self._x)

        # Update step
        y = vec
        self._x = xp + Pp @ inv(Pp + R) @ (y - xp)
        self._P = Pp - Pp @ inv(Pp + R) @ Pp
