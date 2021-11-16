import open3d as o3d
import numpy as np
from copy import copy

class DHparam:
    def __init__(self, d, theta, a, alpha):
        self.d = d
        self.theta = theta
        self.a = a
        self.alpha = alpha

    def __str__(self):
        return "d={}, theta={}, a={}, alpha={}".format(self.d, self.theta, self.a, self.alpha)

    def get_matrix(self):
        c_theta = np.cos(self.theta)
        s_theta = np.sin(self.theta)
        c_alpha = np.cos(self.alpha)
        s_alpha = np.sin(self.alpha)

        return np.array([
            [   c_theta,    -s_theta*c_alpha,   s_theta*s_alpha,    self.a*c_theta  ],
            [   s_theta,    c_theta*c_alpha,    -c_theta*s_alpha,   self.a*s_theta  ],
            [   0,          s_alpha,            c_alpha,            self.d          ],
            [   0,          0,                  0,                  1               ],
        ])

# -------------------------------- Denavit-Hartenberg parameters --------------------------------

# Unit of distance = 1 mm
H1 = 400
H3 = 35
L1 = 25
L2 = 455
L3 = 420
L4 = 80

# Joint angles
theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = 0
theta6 = 0

rest_position_dh = [
    DHparam(-H1,    theta1,             L1, np.pi/2 ), # Frame 0 to 1
    DHparam(0,      theta2,             L2, 0       ), # Frame 1 to 2
    DHparam(0,      theta3 - np.pi/2,   H3, np.pi/2 ), # Frame 2 to 3
    DHparam(-L3,    theta4,             0,  np.pi/2 ), # Frame 3 to 4
    DHparam(0,      theta5,             0,  -np.pi/2), # Frame 4 to 5
    DHparam(-L4,    theta6,             0,  0       ), # Frame 5 to 6
]

# -------------------------------- Constants --------------------------------

# Trihedron and point 3d models
BASIS = o3d.geometry.TriangleMesh.create_coordinate_frame(size=200)

# Link 3d models
LINKS = [
    o3d.geometry.TriangleMesh().create_box(200, 200, 200).translate([-100, -100, -200])
        .paint_uniform_color([0.2, 0.2, 0.2]).compute_vertex_normals(), # Link 0
    o3d.geometry.TriangleMesh().create_box(200, 200, 200).translate([-125, 0, -100])
        .paint_uniform_color([0.8, 0.2, 0.2]).compute_vertex_normals(), # Link 1
    o3d.geometry.TriangleMesh().create_box(455, 100, 100).translate([-455, -50, -50])
        .paint_uniform_color([0.4, 0.8, 0.2]).compute_vertex_normals(), # Link 2
    o3d.geometry.TriangleMesh().create_box(70, 70, 70).translate([-35, -35, -70])
        .paint_uniform_color([0.2, 0.4, 0.6]).compute_vertex_normals(), # Link 3
    o3d.geometry.TriangleMesh().create_box(70, 350, 70).translate([-35, 0, -35])
        .paint_uniform_color([0.9, 0.6, 0.2]).compute_vertex_normals(), # Link 4
    o3d.geometry.TriangleMesh().create_box(50, 50, 50).translate([-25, -25, -50])
        .paint_uniform_color([0.5, 0.1, 0.6]).compute_vertex_normals(), # Link 5
    o3d.geometry.TriangleMesh().create_box(50, 50, 30).translate([-25, -25, 0])
        .paint_uniform_color([0.7, 0.7, 0.7]).compute_vertex_normals(), # Link 6
]

NUM_LINKS = len(LINKS)
NUM_JOINTS = NUM_LINKS - 1

# Global transformation to display things rightside up on screen
G = np.array([
    [1, 0, 0, 0],
    [0, 0, -1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
])

# -------------------------------- Interactive visualization --------------------------------

def mutate_geometry(dst, src):
    dst.vertices = src.vertices
    dst.vertex_normals = src.vertex_normals
    dst.triangle_normals = src.triangle_normals

class State:
    def __init__(self, init_dh):
        self.current_joint = 0
        self.joint_dh = init_dh
        self.transformed_frames = [copy(BASIS) for i in range(NUM_LINKS)]
        self.transformed_links = [copy(LINKS[i]) for i in range(NUM_LINKS)]

    def update_transformations(self):
        # Compose the transformations along the chain
        transformations = [np.eye(4)]
        for dh in self.joint_dh:
            transformations.append(transformations[-1] @ dh.get_matrix())

        # Modify the transformations
        for i in range(NUM_LINKS):
            mutate_geometry(self.transformed_frames[i], copy(BASIS).transform(G @ transformations[i]))
            mutate_geometry(self.transformed_links[i], copy(LINKS[i]).transform(G @ transformations[i]))

    def switch_joint_callback(self, joint):
        def switch_joint_i(_, joint=joint):
            print("Now controlling joint {}".format(joint))
            self.current_joint = joint
            return False
        return switch_joint_i

    def change_angle_callback(self, step):
        def change_angle_x(_, step=step):
            self.joint_dh[self.current_joint].theta += step
            print("DH_{}: {}".format(self.current_joint, self.joint_dh[self.current_joint]))
            self.update_transformations()
            return True
        return change_angle_x

state = State(rest_position_dh)
state.update_transformations()
callbacks = {
    265: state.change_angle_callback(np.pi / 32), # UP to increase a joint angle
    264: state.change_angle_callback(-np.pi / 32) # DOWN to decrease a joint angle  
}
callbacks.update({
    ord('0') + i: state.switch_joint_callback(i) for i in range(NUM_JOINTS) # Number keys to switch joint
})

vis = o3d.visualization.draw_geometries_with_key_callbacks(
    state.transformed_frames + state.transformed_links,
    callbacks,
    width=800, height=600
)
