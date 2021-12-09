import open3d as o3d
import numpy as np
import modern_robotics as mr

# -------------------------------- Power-of-Exponentials parameters --------------------------------

from constants import H1, H3, L1, L2, L3, L4

# Screw axes in frame 0 (global) for joints 1 to 6
screw_axes_0 = [
    np.array([  0,  0,  1,  0,      0,      0,          ]), # S_1
    np.array([  0,  -1, 0,  -H1,    0,      -L1,        ]), # S_2
    np.array([  0,  -1, 0,  -H1,    0,      -L1-L2,     ]), # S_3
    np.array([  -1, 0,  0,  0,      H1+H3,  0,          ]), # S_4
    np.array([  0,  -1, 0,  -H1-H3, 0,      -L1-L2-L3   ]), # S_5
    np.array([  -1, 0,  0,  0,      H1+H3,  0,          ]), # S_6
]

# Screw axes in frame 6 (end effector) for joints 1 to 6 (unused in the code, just for information)
screw_axes_6 = [
    np.array([  -1, 0,  0,  0,          -L1-L2-L3-L4,   0,  ]), # B_1
    np.array([  0,  1,  0,  -L2-L3-L4,  0,              -H3 ]), # B_2
    np.array([  0,  1,  0,  -L3-L4,     0,              -H3 ]), # B_3
    np.array([  0,  0,  1,  0,          0,              0,  ]), # B_4
    np.array([  0,  1,  0,  -L4,        0,              0,  ]), # B_5
    np.array([  0,  0,  1,  0,          0,              0,  ]), # B_6
]

# -------------------------------- Interactive visualization --------------------------------

from visualization import MeshCollection

LINK = lambda i: "Link_{}".format(i)
FRAME = lambda i: "Frame_{}".format(i)
AXIS = lambda i: "Axis_{}".format(i)
BASIS_MESH = o3d.geometry.TriangleMesh.create_coordinate_frame(size=200)
LINE_MESH = o3d.geometry.TriangleMesh.create_cylinder(radius=5, height=400).paint_uniform_color([0, 0, 0])

class State:
    def __init__(self, screw_axes_0, init_transforms, root_transform, link_meshes):
        self.num_links = len(link_meshes)
        self.num_joints = len(screw_axes_0)
        assert(self.num_links == self.num_joints + 1)

        self.current_joint = 0                  # Joint that is being controlled
        self.display_frames = True              # Show the frames on screen
        self.display_robot = True               # Show the robot arm on screen
        self.screw_axes_0 = screw_axes_0        # Screw axes of each joint in frame 0
        self.screw_angles = [0 for i in range(self.num_joints)] # Screw angles of each joint
        self.init_transforms = init_transforms  # Frames 0 to 6 at resting position
        
        self.meshes = MeshCollection(root_transform) # Add the meshes
        for i in range(self.num_links):
            self.meshes.add_mesh(LINK(i), link_meshes[i])   # Show the links 0 to 6
            if i == 0 or i == self.num_links-1:
                self.meshes.add_mesh(FRAME(i), BASIS_MESH)  # Show the frame 0 and 6 (the others are useless in PoE)
        for i in range(self.num_joints):
            self.meshes.add_mesh(AXIS(i+1), LINE_MESH)        # Show the screw axes 1 to 6

        self.do_forward_kinematics() # Solve once to initialize the links positions

    def do_forward_kinematics(self):
        # Compose the transformations along the chain
        transformations = [np.eye(4) for i in range(self.num_links)]
        for i in range(self.num_joints):
            exp_screw_axis_angle = mr.MatrixExp6(mr.VecTose3(self.screw_axes_0[i] * self.screw_angles[i]))
            transformations[i+1] = transformations[i] @ exp_screw_axis_angle

        # Update the meshes
        for i in range(self.num_links):
            T_i = transformations[i] # Transform into frame 0
            M_0i = self.init_transforms[i] # T_0i at resting position
            self.meshes.set_transform(LINK(i), T_i)
            if i == 0 or i == self.num_links-1:
                self.meshes.set_transform(FRAME(i), T_i @ M_0i)
            if i != self.num_links-1:
                self.meshes.set_transform(AXIS(i+1), T_i @ M_0i)

    def switch_joint(self, joint):
        self.current_joint = joint
        print("Now controlling joint {}".format(joint+1))
        return self.meshes.must_update_o3d()

    def change_current_angle(self, step):
        self.screw_angles[self.current_joint] += step
        self.do_forward_kinematics()
        print("theta_{}: {}".format(self.current_joint+1, self.screw_angles[self.current_joint]))
        return self.meshes.must_update_o3d()

    def toggle_robot_display(self):
        self.display_robot = not self.display_robot
        for i in range(self.num_links): self.meshes.set_visibility(LINK(i), self.display_robot)
        return self.meshes.must_update_o3d()

    def toggle_frame_display(self):
        self.display_frames = not self.display_frames
        self.meshes.set_visibility(FRAME(0), self.display_frames)
        self.meshes.set_visibility(FRAME(self.num_links-1), self.display_frames)
        for i in range(self.num_joints): self.meshes.set_visibility(AXIS(i+1), self.display_frames)
        return self.meshes.must_update_o3d()


if __name__ == "__main__":
    from constants import INIT_TRANSFORMS, ROOT_TRANSFORM, LINK_MESHES

    state = State(screw_axes_0, INIT_TRANSFORMS, ROOT_TRANSFORM, LINK_MESHES)
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
