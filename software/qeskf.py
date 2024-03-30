from numpy import array, block, eye, zeros, trace
from numpy.linalg import inv, norm, det
from scipy.linalg import expm
from numpyx import vec3, vec4, qmull, qrotv, xmat, rmat3q, rmat4q
from math import sin, cos, radians

class QuaternionESKF:
    def __init__(self, gn, mn):
        self._gn = gn
        self._mn = mn

        self._q = vec4(1.0, 0.0, 0.0, 0.0)
        self._P = eye(3) #zeros((3, 3))
        self._acc_var = 0.0

    @property
    def matrix(self):
        return rmat4q(self._q)

    @property
    def covariance(self):
        return self._P

    def step(self, dt, acc, mag, rot):
        # Form process noise covariance matrix.
        Q = eye(3) * 0.01 * dt

        # Form measurement covariance matrix.
        self._acc_var = 0.9 * self._acc_var + 0.1 * (0.1 + 4.0 * abs(1.0 - acc.T @ acc) ** 2)
        self._var = 1.0
        R = block([
                [self._acc_var*1*eye(3), zeros((3,3))],
                [zeros((3,3)), self._var*eye(3)]
            ])

        # Prediction step
        qp = qmull(self._q, qrotv(dt * rot))

        # Markley & Crassidis
        Fx = expm(-dt * xmat(rot))
        Pp = Fx @ self._P @ Fx.T + Q

        # Kok
        # Fx = (dt / 2)**2 * rot @ rot.T + (eye(3) - (dt / 2) * xmat(rot)) @ (eye(3) - (dt / 2) * xmat(rot))
        # Fw = -dt * eye(3) + (dt / 2)**2 * xmat(rot)
        # Pp = Fx @ self._P @ Fx.T + Fw @ (5 * eye(3) * dt) @ Fw.T

        # Update step
        y = block([[acc], [mag]])
        Rp = rmat3q(qp).T
        yp = block([[-Rp @ self._gn], [Rp @ self._mn]])
        Hx = block([[-xmat(Rp @ self._gn)], [xmat(Rp @ self._mn)]])
        x = Pp @ Hx.T @ inv(Hx @ Pp @ Hx.T + R) @ (y - yp)
        Pt = Pp - Pp @ Hx.T @ inv(Hx @ Pp @ Hx.T + R) @ Hx @ Pp

        # Reset step
        self._q = qmull(qp, qrotv(x))
#        self._P = Pt
        J = eye(3) - (1/2) * xmat(x)
        self._P = J @ Pt @ J.T
