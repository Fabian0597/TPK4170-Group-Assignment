from copy import copy
import numpy as np
import open3d as o3d

EMPTY = o3d.geometry.TriangleMesh()

class MeshCollectionElement:
    def __init__(self, mesh):
        self.untransformed_mesh = copy(mesh)
        self.transformation = np.eye(4)
        self.transformed_mesh = copy(mesh)
        self.visible = True

class MeshCollection:
    def __init__(self, root_T):
        self.root_T = root_T
        self.objects = {}
        self.must_update = False

    def add_mesh(self, name, o3d_mesh):
        self.must_update = True
        self.objects[name] = MeshCollectionElement(o3d_mesh)

    def set_transform(self, name, T):
        self.must_update = True
        x = self.objects[name]
        x.transformation = T
        if x.visible:
            new_transformed = copy(x.untransformed_mesh).transform(self.root_T @ T)
            x.transformed_mesh.vertices = new_transformed.vertices
            x.transformed_mesh.vertex_normals = new_transformed.vertex_normals
            x.transformed_mesh.triangle_normals = new_transformed.triangle_normals

    def get_meshes(self):
        return [x.transformed_mesh for x in self.objects.values()]

    def must_update_o3d(self):
        if self.must_update:
            self.must_update = False
            return True
        else:
            return False

    def set_visibility(self, name, visible):
        self.must_update = True
        x = self.objects[name]
        if visible:
            x.visible = True
            self.set_transform(name, x.transformation)
        else:
            x.visible = False
            x.transformed_mesh.vertices = EMPTY.vertices
            x.transformed_mesh.vertex_normals = EMPTY.vertex_normals
            x.transformed_mesh.triangle_normals = EMPTY.triangle_normals
