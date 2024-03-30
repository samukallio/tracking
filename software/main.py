import time
import sys
import os
import pygame
import serial

from math import pi, sin, cos, acos, degrees, radians, sqrt
from numpy import array, block, zeros, float32, concatenate
from numpy.linalg import det, norm
from numpyx import vec3, vec4, tmatxyz, rmatxyz, xmat, mat3to4, mat2wnd

from graphics import Renderer, Graph
from graphics import create_arrow_mesh, create_sphere_mesh, load_image_texture
from qeskf import QuaternionESKF
from vekf import VectorEKF
from qint import QuaternionIntegrator

# -----------------------------------------------------------------------------

NAV_G = vec3(0.0, 0.0, -1.0)
NAV_M = vec3(0.0, sin(radians(17)), -cos(radians(17)))

arrow_mesh = None
sphere_mesh = None
background = None

qeskf = QuaternionESKF(gn = NAV_G, mn = NAV_M)
qint = QuaternionIntegrator()

mag_vekf = VectorEKF(1.0)
acc_vekf = VectorEKF(1.0)

outfile = None

cov_graph = Graph()
accx_graph, accy_graph, accz_graph = Graph(), Graph(), Graph()
magx_graph, magy_graph, magz_graph = Graph(), Graph(), Graph()
rotx_graph, roty_graph, rotz_graph = Graph(), Graph(), Graph()

# -----------------------------------------------------------------------------

def draw_triad(g, rotation):
    """Draw an XYZ-triad using red, green and blue arrows."""
    # X-axis arrow
    transform = rotation @ rmatxyz(0,pi/2,0)
    tint = vec4(1.0, 0.0, 0.0, 0.0)
    g.draw_mesh(arrow_mesh, transform, tint)

    # Y-axis arrow
    transform = rotation @ rmatxyz(-pi/2,0,0)
    tint = vec4(0.0, 1.0, 0.0, 0.0)
    g.draw_mesh(arrow_mesh, transform, tint)

    # Z-axis arrow
    transform = rotation
    tint = vec4(0.0, 0.0, 1.0, 0.0)
    g.draw_mesh(arrow_mesh, transform, tint)

    # Origin widget
    transform = rotation
    tint = vec4(1.0, 1.0, 1.0, 0.0)
    g.draw_mesh(sphere_mesh, transform, tint)

def orthonormalize(g, m):
    """Create right-handed orthonormal frame out of g and m."""
    ez = -g / (g.T @ g)**0.5
    ex = xmat(m) @ ez
    ex /= (ex.T @ ex)**0.5
    ey = xmat(ez) @ ex
    return mat3to4(block([[ ex, ey, ez ]]).T)

def loop(g, dt, acc, mag, rot):
    global outfile

    # Set world-to-view matrix for mesh rendering.
    g.set_view_matrix(tmatxyz(0, 0, -5) @ rmatxyz(-pi/2,0,0))

    # Quaternion ESKF.
    qeskf.step(dt, acc, mag, rot)
    g.set_viewport(0, 0, 320, 320)
    qeskf_mat = qeskf.matrix
    draw_triad(g, qeskf_mat)

    # Vector EKFs.
    acc_vekf.step(dt, acc, rot)
    mag_vekf.step(dt, mag, rot)
    g.set_viewport(320, 0, 320, 320)
    vkf_mat = orthonormalize(-acc_vekf.vector, mag_vekf.vector)
    draw_triad(g, vkf_mat)

    # Acc./Mag. only.
    g.set_viewport(0, 300, 320, 320)
    accmag_mat = orthonormalize(-acc, mag)
    draw_triad(g, accmag_mat)

    # Gyroscope only.
    qint.step(dt, rot)
    g.set_viewport(320, 300, 320, 320)
    gyro_mat = qint.matrix
    draw_triad(g, gyro_mat)

    qeskf_elems = qeskf_mat[0:3,0:3].flatten()
    vkf_elems = vkf_mat[0:3,0:3].flatten()
    accmag_elems = accmag_mat[0:3,0:3].flatten()
    gyro_elems = gyro_mat[0:3,0:3].flatten()
    elems = concatenate((accmag_elems, gyro_elems, qeskf_elems, vkf_elems))
    if outfile:
        outfile.write(" ".join(map(str, elems)) + "\n")

    # Update graphs.
    cov_graph.add(det(qeskf.covariance) ** 0.5)
#    cov_graph.add(norm(qeskf.covariance, 'fro'))
#    print(norm(qeskf.covariance, 'fro'))
    accx_graph.add(acc[0][0])
    accy_graph.add(acc[1][0])
    accz_graph.add(acc[2][0])
    magx_graph.add(mag[0][0])
    magy_graph.add(mag[1][0])
    magz_graph.add(mag[2][0])
    rotx_graph.add(rot[0][0] / (4 * pi))
    roty_graph.add(rot[1][0] / (4 * pi))
    rotz_graph.add(rot[2][0] / (4 * pi))

    # Draw graphs.
    def window(i, j=0):
        x0 = (11 + 210*j - 320) / 320.0
        x1 = (199 + 210*j - 320) / 320.0
        y0 = (20 + 160*i - 320) / 320.0
        y1 = (140 + 160*i - 320) / 320.0
        return mat2wnd(x0, y0, x1, y1, sy=0.5)

    g.set_viewport(640,0,640,640)
    g.draw_graph(cov_graph, vec4(1,1,1,1), mat2wnd(-0.96875, 0.1875, 0.9375, 0.9375, sy=50))
    g.draw_graph(accx_graph, vec4(1,0,0,1), window(2,0))
    g.draw_graph(accy_graph, vec4(0,1,0,1), window(1,0))
    g.draw_graph(accz_graph, vec4(0,0,1,1), window(0,0))
    g.draw_graph(magx_graph, vec4(1,0,0,1), window(2,1))
    g.draw_graph(magy_graph, vec4(0,1,0,1), window(1,1))
    g.draw_graph(magz_graph, vec4(0,0,1,1), window(0,1))
    g.draw_graph(rotx_graph, vec4(1,0,0,1), window(2,2))
    g.draw_graph(roty_graph, vec4(0,1,0,1), window(1,2))
    g.draw_graph(rotz_graph, vec4(0,0,1,1), window(0,2))

def acc_model(acc):
    """Accelerometer calibration model."""
    accb = vec3(-547.0262, -96.7392, 92.4361)
    accg = vec3(16421, 16454, 16611) / 2
    return (acc - accb) / accg

def mag_model(mag):
    """Magnetometer calibration model."""
    magr = array([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ])
    magb = vec3(78.1810, 60.9789, -21.9482)
    magg = vec3(323.9201, 320.9182, 321.4008)
    return magr @ (mag - magb) / magg

def rot_model(rot):
    """Gyroscope calibration model."""
    rotb = vec3(-0.6874, -39.7461, -19.8377)

    return (rot - rotb) * (radians(1000) / 32768.0)

def main():
    global arrow_mesh, sphere_mesh, background, outfile

    g = Renderer(1280, 640, "Orientation tracking")

    # Create meshes.
    arrow_mesh = create_arrow_mesh(g)
    sphere_mesh = create_sphere_mesh(g, 0.1)

    # Load background
    background = load_image_texture(g, "layout.png")

    t0 = time.time(); t1 = t0

    usb0 = serial.Serial('COM5', baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

    while True:
        # Handle window events.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_ESCAPE:
                    return
                if event.key == pygame.K_g:
                    qint._q = qeskf._q
                if event.key == pygame.K_r:
                    qeskf._q = vec4(1.0, 0.0, 0.0, 0.0)
                    mag_vekf._x = NAV_M
                    acc_vekf._x = -NAV_G
                if event.key == pygame.K_q:
                    for i in range(1,10000):
                        fname = "output/data%d.txt" % i
                        if os.path.exists(fname):
                            continue
                        if outfile:
                            print("Stopped recording")
                            outfile.close()
                        outfile = open(fname, 'w')
                        print("Started recording to %s..." % fname)
                        break
                if event.key == pygame.K_w:
                    print("Stopped recording")
                    if outfile:
                        outfile.close()
                    outfile = None

        # Read raw measurement data from the board.
        while True:
            line = usb0.readline()
            if line:
                fields = line.strip().split(b' ')
                if len(fields) != 9:
                    continue
                values = list(map(float, fields))
                break

        # Apply sensor bias/scaling models to obtain normalized sensor data.
        acc = acc_model(vec3(*values[0:3]))
        mag = mag_model(vec3(*values[3:6]))
        rot = rot_model(vec3(*values[6:9]))

        # Compute previous frame start and end times.
        t0, t1 = t1, time.time()

        # Draw background.
        g.clear()
        g.set_viewport(0, 0, 1280, 640)
        g.draw_texture(background, -1, -1, 1, 1)

        # Run the filters, draw UI.
        loop(g, t1 - t0, acc, mag, rot)

        # Show the results.
        pygame.display.flip()

if __name__ == '__main__':
    try:
        pygame.init()
        main()
    finally:
        pygame.quit()
