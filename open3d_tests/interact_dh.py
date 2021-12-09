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

from constants import H1, H3, L1, L2, L3, L4

# The joints will be refered to with an index from 0 to 5 instead of 1 to 6. Deal with it
init_dh = [
    DHparam(-H1,    0,          L1, np.pi/2 ), # Frame 0 to 1
    DHparam(0,      0,          L2, 0       ), # Frame 1 to 2
    DHparam(0,      -np.pi/2,   H3, np.pi/2 ), # Frame 2 to 3
    DHparam(-L3,    0,          0,  -np.pi/2), # Frame 3 to 4
    DHparam(0,      0,          0,  np.pi/2 ), # Frame 4 to 5
    DHparam(-L4,    0,          0,  0       ), # Frame 5 to 6
]

# -------------------------------- Interactive visualization --------------------------------

from visualization import MeshCollection

LINK = lambda i: "Link_{}".format(i)
FRAME = lambda i: "Frame_{}".format(i)
BASIS_MESH = o3d.geometry.TriangleMesh.create_coordinate_frame(size=200)

class State:
    def __init__(self, init_dh, init_transforms, root_transform, link_meshes):
        self.num_links = len(link_meshes)
        self.num_joints = len(init_dh)
        assert(self.num_links == self.num_joints + 1)

        self.current_joint = 0                  # Joint that is being controlled
        self.display_frames = True              # Show the frames on screen
        self.display_robot = True               # Show the robot arm on screen
        self.joint_dh = init_dh                 # DH parameters of each joints
        self.init_transforms = init_transforms  # Frames 0 to 6 at resting position

        self.meshes = MeshCollection(root_transform)
        for i in range(self.num_links):
            self.meshes.add_mesh(FRAME(i), BASIS_MESH)      # Show the fames 0 to 6
            self.meshes.add_mesh(LINK(i), link_meshes[i])   # Show the links 0 to 6

        self.do_forward_kinematics() # Solve once to initialize the links positions

    def do_forward_kinematics(self):
        # Compose the transformations along the chain
        transformations = [np.eye(4) for i in range(self.num_links)]
        for i in range(self.num_joints):
            transformations[i+1] = transformations[i] @ self.joint_dh[i].get_matrix()

        # Update the meshes
        for i in range(self.num_links):
            T_0i = transformations[i] # Change of frame from i to 0
            inv_M_0i = np.linalg.inv(self.init_transforms[i]) # Inverse of T_0i at resting position
            self.meshes.set_transform(FRAME(i), T_0i)
            self.meshes.set_transform(LINK(i), T_0i @ inv_M_0i)

    def switch_joint(self, joint):
        self.current_joint = joint
        print("Now controlling joint {}".format(joint+1))
        return self.meshes.must_update_o3d()

    def change_current_angle(self, step):
        self.joint_dh[self.current_joint].theta += step
        self.do_forward_kinematics()
        print("DH_{}: {}".format(self.current_joint+1, self.joint_dh[self.current_joint]))
        return self.meshes.must_update_o3d()

    def toggle_robot_display(self):
        self.display_robot = not self.display_robot
        for i in range(self.num_links): self.meshes.set_visibility(LINK(i), self.display_robot)
        return self.meshes.must_update_o3d()

    def toggle_frame_display(self):
        self.display_frames = not self.display_frames
        for i in range(self.num_links): self.meshes.set_visibility(FRAME(i), self.display_frames)
        return self.meshes.must_update_o3d()


if __name__ == "__main__":
    from constants import INIT_TRANSFORMS, ROOT_TRANSFORM, LINK_MESHES

    state = State(init_dh, INIT_TRANSFORMS, ROOT_TRANSFORM, LINK_MESHES)
    callbacks = {
        265: lambda _ : state.change_current_angle(np.pi / 32),     # Press UP to increase a joint angle
        264: lambda _ : state.change_current_angle(-np.pi / 32),    # Press DOWN to decrease a joint angle  
        ord('R'): lambda _ : state.toggle_robot_display(),          # PressR to show or hide the robot
        ord('F'): lambda _ : state.toggle_frame_display(),          # F to show or hide the frames
    }
    callbacks.update({
        # Press the number keys to switch joint
        ord('1') + i: lambda _, i=i : state.switch_joint(i) for i in range(state.num_joints)
    })

    vis = o3d.visualization.draw_geometries_with_key_callbacks(
        state.meshes.get_meshes(), callbacks, width=800, height=600
    )
