import OpenGL.GL as GL
import OpenGL.GL.shaders
import ctypes
import pygame, pygame.locals
import numpy as np
import numpyx as npx
from math import *

class Graph:
    def __init__(self):
        self._values = np.zeros(512, dtype=np.float32)

    def add(self, value):
        self._values = np.roll(self._values, -1)
        self._values[-1] = value

class Mesh:
    def __init__(self, context, vao, ntris):
        self.context = context
        self.vao = vao
        self.ntris = ntris

class Texture:
    def __init__(self, texture):
        self.texture = texture

class Renderer:
    _MESH_VERTEX_SHADER = """
        #version 130

        in vec4 position;
        in vec4 normal;
        in vec4 color;
        uniform mat4 WorldFromObject;
        uniform mat4 ViewFromWorld;
        uniform mat4 ScreenFromView;
        uniform vec4 TintColor;

        void main()
        {
            float refl = abs(dot(ViewFromWorld * WorldFromObject * normal, vec4(0.0,0.0,-1.0,0.0))) * 0.8 + 0.2;

            gl_Position = ScreenFromView * ViewFromWorld * WorldFromObject * position;
            gl_FrontColor = refl * TintColor * color;
        }
        """

    _MESH_FRAGMENT_SHADER = """
        #version 130

        void main()
        {
           gl_FragColor = gl_Color;
        }
        """

    _POLYLINE_VERTEX_SHADER = """
        #version 130

        in float time;
        in float value;
        uniform vec4 Color;
        uniform mat4 Transform;

        void main()
        {
            gl_Position = Transform * vec4(time, value, 0.0, 1.0);
            gl_FrontColor = Color;
        }
        """

    _POLYLINE_FRAGMENT_SHADER = """
        #version 130

        void main()
        {
           gl_FragColor = gl_Color;
        }
        """

    _BLIT_VERTEX_SHADER = """
        #version 130

        in vec2 v_xy;
        in vec2 v_uv;

        out vec2 f_uv;

        void main()
        {
            gl_Position = vec4(v_xy, 0.0, 1.0);
            f_uv = v_uv;
        }
        """

    _BLIT_FRAGMENT_SHADER = """
        #version 130

        uniform sampler2D sampler;
        in vec2 f_uv;

        void main()
        {
           gl_FragColor = vec4(texture(sampler, f_uv.xy).rgb, 1.0);
        }
        """

    def __init__(self, width, height, title):
        pygame.display.gl_set_attribute(pygame.locals.GL_MULTISAMPLEBUFFERS,1)
        pygame.display.set_caption(title)
        self._screen = pygame.display.set_mode((width, height), pygame.OPENGL|pygame.DOUBLEBUF)

        GL.glClearColor(0.2, 0.2, 0.2, 1.0)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_LINE_SMOOTH)
        GL.glLineWidth(1.5)

        self._object_shader = OpenGL.GL.shaders.compileProgram(
            OpenGL.GL.shaders.compileShader(self._MESH_VERTEX_SHADER, GL.GL_VERTEX_SHADER),
            OpenGL.GL.shaders.compileShader(self._MESH_FRAGMENT_SHADER, GL.GL_FRAGMENT_SHADER)
        )

        self._graph_shader = OpenGL.GL.shaders.compileProgram(
            OpenGL.GL.shaders.compileShader(self._POLYLINE_VERTEX_SHADER, GL.GL_VERTEX_SHADER),
            OpenGL.GL.shaders.compileShader(self._POLYLINE_FRAGMENT_SHADER, GL.GL_FRAGMENT_SHADER)
        )

        self._blit_shader = OpenGL.GL.shaders.compileProgram(
            OpenGL.GL.shaders.compileShader(self._BLIT_VERTEX_SHADER, GL.GL_VERTEX_SHADER),
            OpenGL.GL.shaders.compileShader(self._BLIT_FRAGMENT_SHADER, GL.GL_FRAGMENT_SHADER)
        )

        self._blit_vbo = GL.glGenBuffers(1)
        self._blit_vao = GL.glGenVertexArrays(1)

        GL.glBindVertexArray(self._blit_vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._blit_vbo)
        index = GL.glGetAttribLocation(self._blit_shader, 'v_xy')
        GL.glEnableVertexAttribArray(index)
        GL.glVertexAttribPointer(index, 2, GL.GL_FLOAT, False, 16, ctypes.c_void_p(0))
        index = GL.glGetAttribLocation(self._blit_shader, 'v_uv')
        GL.glEnableVertexAttribArray(index)
        GL.glVertexAttribPointer(index, 2, GL.GL_FLOAT, False, 16, ctypes.c_void_p(8))
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        GL.glBindVertexArray(0)

        self._graph_time_vbo = GL.glGenBuffers(1)
        self._graph_value_vbo = GL.glGenBuffers(1)
        self._graph_vao = GL.glGenVertexArrays(1)

        GL.glBindVertexArray(self._graph_vao)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._graph_time_vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, 4*512, np.linspace(-1,1,512,dtype=np.float32), GL.GL_STATIC_DRAW)
        index = GL.glGetAttribLocation(self._graph_shader, 'time')
        GL.glEnableVertexAttribArray(index)
        GL.glVertexAttribPointer(index, 1, GL.GL_FLOAT, False, 0, ctypes.c_void_p(0))
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._graph_value_vbo)
        index = GL.glGetAttribLocation(self._graph_shader, 'value')
        GL.glEnableVertexAttribArray(index)
        GL.glVertexAttribPointer(index, 1, GL.GL_FLOAT, False, 0, ctypes.c_void_p(0))
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)
        GL.glBindVertexArray(0)

        self.set_viewport(0,0,512,512)

    def set_viewport(self, x, y, w, h):
        zn = 100.0
        zf = 1000.0
        pw = min(w / h, 1.0)
        ph = min(h / w, 1.0)

        self._projection_matrix = np.array([
                2*zn/pw, 0, 0, 0,
                0, 2*zn/ph, 0, 0,
                0, 0, zf/(zn-zf), -1,
                0, 0, zn*zf/(zn-zf), 0
            ], dtype=np.float32)

        GL.glViewport(x, y, w, h)

    def set_view_matrix(self, matrix):
        self._view_matrix = matrix

    def create_texture(self, width, height, data):
        texture = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, texture)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_S, GL.GL_CLAMP)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_WRAP_T, GL.GL_CLAMP)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, GL.GL_RGB, width, height, 0, GL.GL_RGB, GL.GL_UNSIGNED_BYTE, data)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

        return Texture(texture)

    def create_mesh(self, vertices):
        vao = GL.glGenVertexArrays(1)
        GL.glBindVertexArray(vao)

        data = np.array(vertices, dtype=np.float32)

        vbo = GL.glGenBuffers(1)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, len(data) * 4, data, GL.GL_STATIC_DRAW)

        loc = GL.glGetAttribLocation(self._object_shader, 'position')
        GL.glEnableVertexAttribArray(loc)
        GL.glVertexAttribPointer(loc, 4, GL.GL_FLOAT, False, 48, ctypes.c_void_p(0))

        loc = GL.glGetAttribLocation(self._object_shader, 'normal')
        GL.glEnableVertexAttribArray(loc)
        GL.glVertexAttribPointer(loc, 4, GL.GL_FLOAT, False, 48, ctypes.c_void_p(16))

        loc = GL.glGetAttribLocation(self._object_shader, 'color')
        GL.glEnableVertexAttribArray(loc)
        GL.glVertexAttribPointer(loc, 4, GL.GL_FLOAT, False, 48, ctypes.c_void_p(32))

        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

        GL.glBindVertexArray(0)

        return vao

    def draw_texture(self, texture, x0, y0, x1, y1):
        GL.glUseProgram(self._blit_shader)

        index = GL.glGetUniformLocation(self._blit_shader, "sampler");
        GL.glProgramUniform1i(self._blit_shader, index, 0);
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, texture.texture)

        vertices = np.array([
            x0, y0, 0, 1,
            x1, y0, 1, 1,
            x1, y1, 1, 0,
            x0, y0, 0, 1,
            x1, y1, 1, 0,
            x0, y1, 0, 0
        ], dtype=np.float32)

        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._blit_vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, 4*len(vertices), vertices, GL.GL_STREAM_DRAW);
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

        GL.glBindVertexArray(self._blit_vao)
        GL.glDepthMask(GL.GL_FALSE)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 6)
        GL.glDepthMask(GL.GL_TRUE)
        GL.glBindVertexArray(0)

        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

        GL.glUseProgram(0)

    def draw_mesh(self, obj, xform, tint=npx.vec4(1.0,1.0,1.0,1.0)):
        GL.glUseProgram(self._object_shader)

        index = GL.glGetUniformLocation(self._object_shader, 'WorldFromObject')
        GL.glUniformMatrix4fv(index, 1, True, xform)

        index = GL.glGetUniformLocation(self._object_shader, 'ViewFromWorld')
        GL.glUniformMatrix4fv(index, 1, True, self._view_matrix)

        index = GL.glGetUniformLocation(self._object_shader, 'ScreenFromView')
        GL.glUniformMatrix4fv(index, 1, True, self._projection_matrix)

        index = GL.glGetUniformLocation(self._object_shader, 'TintColor')
        GL.glUniform4fv(index, 1, tint)

        GL.glBindVertexArray(obj.vao)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, obj.ntris*3)
        GL.glBindVertexArray(0)

        GL.glUseProgram(0)

    def draw_graph(self, graph, color, transform=np.eye(4)):
        GL.glUseProgram(self._graph_shader)

        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._graph_value_vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, 4*512, graph._values, GL.GL_STREAM_DRAW);
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, 0)

        index = GL.glGetUniformLocation(self._graph_shader, 'Color')
        GL.glUniform4fv(index, 1, color)

        index = GL.glGetUniformLocation(self._graph_shader, 'Transform')
        GL.glUniformMatrix4fv(index, 1, True, transform)

        GL.glBindVertexArray(self._graph_vao)
        GL.glDrawArrays(GL.GL_LINE_STRIP, 0, 512)
        GL.glBindVertexArray(0)

        GL.glUseProgram(0)

    def clear(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

def lerp(t, a, b):
    return (1 - t) * a + t * b

def _make_frustum(segments, z1, r1, z2, r2):
    vertices = []

    angle = 360.0 / segments

    for k in range(segments):
        phi0 = radians(k*angle)
        phi1 = radians((k+1)*angle)
        theta = atan2(r1 - r2, z2 - z1)

        if r2 > 0:
            vertices.extend([
                r2*cos(phi0), r2*sin(phi0), z2, 1.0,
                cos(phi0)*cos(theta), sin(phi0)*cos(theta), sin(theta), 0.0,
                1.0, 1.0, 1.0, 0.0,

                r1*cos(phi0), r1*sin(phi0), z1, 1.0,
                cos(phi0)*cos(theta), sin(phi0)*cos(theta), sin(theta), 0.0,
                1.0, 1.0, 1.0, 0.0,

                r2*cos(phi1), r2*sin(phi1), z2, 1.0,
                cos(phi1)*cos(theta), sin(phi1)*cos(theta), sin(theta), 0.0,
                1.0, 1.0, 1.0, 0.0
            ])

        if r1 > 0:
            vertices.extend([
                r2*cos(phi1), r2*sin(phi1), z2, 1.0,
                cos(phi1)*cos(theta), sin(phi1)*cos(theta), sin(theta), 0.0,
                1.0, 1.0, 1.0, 0.0,

                r1*cos(phi0), r1*sin(phi0), z1, 1.0,
                cos(phi0)*cos(theta), sin(phi0)*cos(theta), sin(theta), 0.0,
                1.0, 1.0, 1.0, 0.0,

                r1*cos(phi1), r1*sin(phi1), z1, 1.0,
                cos(phi1)*cos(theta), sin(phi1)*cos(theta), sin(theta), 0.0,
                1.0, 1.0, 1.0, 0.0
            ])

    return vertices

def create_sphere_mesh(context, radius):
    segments = 24
    angle = 180.0 / segments

    vertices = []

    for k in range(segments):
        theta1 = radians(k*angle)
        theta2 = radians((k+1)*angle)

        r1 = radius * sin(theta1)
        r2 = radius * sin(theta2)
        z1 = radius * -cos(theta1)
        z2 = radius * -cos(theta2)

        vertices.extend(_make_frustum(segments, z1, r1, z2, r2))

    vao = context.create_mesh(vertices)
    ntris = len(vertices) // 36

    return Mesh(context, vao, ntris)

def create_arrow_mesh(context, bottom=0.0, top=2.0):
    sides = 24

    vertices = []

    middle = lerp(0.8, bottom, top)

    parts = [
        (bottom, 0.00, bottom, 0.05),
        (bottom, 0.05, middle, 0.05),
        (middle, 0.05, middle, 0.20),
        (middle, 0.20, top,    0.00)
    ]

    for z1, r1, z2, r2 in parts:
        vertices.extend(_make_frustum(sides, z1, r1, z2, r2))

    vao = context.create_mesh(vertices)
    ntris = len(vertices) // 36

    return Mesh(context, vao, ntris)

def load_image_texture(context, path):
    return context.create_texture(1280, 640,
        pygame.image.tostring(pygame.image.load(path), 'RGB'))
