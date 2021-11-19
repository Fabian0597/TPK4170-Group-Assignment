import open3d as o3d
import numpy as np

# Unit of distance = 1 mm
H1 = 400
H3 = 35
L1 = 25
L2 = 455
L3 = 420
L4 = 80

# Load the 3d models of the links
robot_model = o3d.io.read_triangle_model("robot_model.dae")
robot_model = {
    x.mesh_name : x.mesh.scale(10, [0, 0, 0]).compute_vertex_normals()
    .paint_uniform_color(robot_model.materials[x.material_idx].base_color[:3]**0.5) for x in robot_model.meshes
}
LINK_MESHES = [
    robot_model["Link_0_mesh"],
    robot_model["Link_1_mesh"],
    robot_model["Link_2_mesh"],
    robot_model["Link_3_mesh"],
    robot_model["Link_4_mesh"],
    robot_model["Link_5_mesh"],
    robot_model["Link_6_mesh"],
]

# Frame 0 in the referential of open3d. This is necessary do display things rightside up
ROOT_TRANSFORM = np.array([
    [1, 0, 0, 0],
    [0, 0, -1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
])

# Frames 0 to 6 in the DH convention at resting position (when all joints are 0)
INIT_TRANSFORMS = [
    np.eye(4), # M_0
    np.array([ # M_1
        [1, 0, 0, L1],
        [0, 0, -1, 0],
        [0, 1, 0, -H1],
        [0, 0, 0, 1],
    ]),
    np.array([ # M_2
        [1, 0, 0, L1+L2],
        [0, 0, -1, 0],
        [0, 1, 0, -H1],
        [0, 0, 0, 1],
    ]),
    np.array([ # M_3
        [0, 0, -1, L1+L2],
        [0, -1, 0, 0],
        [-1, 0, 0, -H1-H3],
        [0, 0, 0, 1],
    ]),
    np.array([ # M_4
        [0, 1, 0, L1+L2+L3],
        [0, 0, -1, 0],
        [-1, 0, 0, -H1-H3],
        [0, 0, 0, 1],
    ]),
    np.array([ # M_5
        [0, 0, -1, L1+L2+L3],
        [0, -1, 0, 0],
        [-1, 0, 0, -H1-H3],
        [0, 0, 0, 1],
    ]),
    np.array([ # M_6, a.k.a M
        [0, 0, -1, L1+L2+L3+L4],
        [0, -1, 0, 0],
        [-1, 0, 0, -H1-H3],
        [0, 0, 0, 1],
    ]),
]
