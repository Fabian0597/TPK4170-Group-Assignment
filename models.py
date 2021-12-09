# Copyright 2021 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import numpy as np
import pythreejs
from pythreejs import (
    Group,
    Mesh,
    Geometry,
    SphereGeometry,
    BufferGeometry,
    PlaneGeometry,
    BufferAttribute,
    Points,
    PointsMaterial,
    LineBasicMaterial,
    MeshLambertMaterial,
    MeshPhongMaterial,
    MeshBasicMaterial,
)

from matplotlib.colors import to_hex
from collada import Collada


class Grid(Group):
    def __init__(self, num_cells=5, color="#cccccc", linewidth=1, cellsize=0.5):
        Group.__init__(self)
        material = LineBasicMaterial(color=color, linewidth=linewidth)
        for i in range(num_cells + 1):
            edge = cellsize * num_cells / 2
            position = edge - (i * cellsize)
            geometry_h = Geometry(vertices=[(-edge, position, 0), (edge, position, 0)])
            geometry_v = Geometry(vertices=[(position, -edge, 0), (position, edge, 0)])
            self.add(pythreejs.Line(geometry=geometry_h, material=material))
            self.add(pythreejs.Line(geometry=geometry_v, material=material))


class Ball(Mesh):
    def __init__(self, color="red", radius=0.025):
        Mesh.__init__(
            self,
            geometry=SphereGeometry(radius=radius),
            material=MeshLambertMaterial(color=color),
        )


class ColladaMesh(Group):
    def __init__(self, filename, scale=1.0):
        Group.__init__(self)

        self._dae = Collada(filename)
        self._load_mesh(self._dae, scale=scale)

    def _load_mesh(self, dae, scale):
        materials = self._load_material(dae)
        for geometry in dae.geometries:
            for primitive in geometry.primitives:
                vertices = primitive.vertex[primitive.vertex_index] * scale
                normals = primitive.normal[primitive.normal_index]
                buffer_geometry = BufferGeometry(
                    attributes={
                        "position": BufferAttribute(array=vertices),
                        "normal": BufferAttribute(array=normals),
                    }
                )
                material = materials[primitive.material]
                mesh = Mesh(geometry=buffer_geometry, material=material)
                self.add(mesh)

    def _load_material(self, dae):
        materials = {}
        for material in dae.materials:
            name = material.id
            color = to_hex(material.effect.diffuse)
            # specular = to_hex(material.effect.specular)
            specular = to_hex([1, 1, 1, 1])
            materials[name] = MeshPhongMaterial(
                color=color, specular=specular, shininess=30
            )
        return materials


class Plane(Mesh):
    def __init__(self, color="pink", transparent=False):
        Mesh.__init__(
            self, geometry=PlaneGeometry(), material=MeshBasicMaterial(color=color)
        )
        self.material.side = "DoubleSide"
        if transparent:
            self.material.transparent = transparent
            self.material.opacity = 0.5


class Line(pythreejs.Line):
    def __init__(self, start, end, color="white", linewidth=1):
        geometry = BufferGeometry(
            attributes={
                "position": BufferAttribute(
                    np.vstack((start, end)).astype(np.float32), normalized=False
                )
            }
        )
        material = LineBasicMaterial(color=color, linewidth=linewidth)
        pythreejs.Line.__init__(self, geometry=geometry, material=material)


class Triangle(Mesh):
    def __init__(self, p1, p2, p3, color="yellow"):
        geometry = BufferGeometry(
            attributes={
                "position": BufferAttribute(
                    np.vstack((p1, p2, p3)).reshape(3, 3).astype(np.float32),
                    normalized=False,
                )
            }
        )
        material = MeshBasicMaterial(color=color)
        material.side = "DoubleSide"
        Mesh.__init__(self, geometry=geometry, material=material)


class PointCloud(Points):
    def __init__(self, points, point_size=0.001, color="green"):
        geometry = BufferGeometry(
            attributes={"position": BufferAttribute(array=points.astype(np.float32))}
        )
        material = PointsMaterial(size=point_size, color=color)
        Points.__init__(self, geometry=geometry, material=material)


class Trihedron(Group):
    def __init__(self, size=1, linewidth=1):
        Group.__init__(self)

        self.add(Line([0, 0, 0], [size, 0, 0], "red", linewidth))
        self.add(Line([0, 0, 0], [0, size, 0], "green", linewidth))
        self.add(Line([0, 0, 0], [0, 0, size], "blue", linewidth))