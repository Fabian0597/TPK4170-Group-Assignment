import open3d as o3d
import numpy as np
import interact_dh as dh
import interact_poe as poe

if __name__ == "__main__":
    from constants import INIT_TRANSFORMS, ROOT_TRANSFORM, LINK_MESHES

    # Translate the roots to visualize the robots side by side
    dh_root = ROOT_TRANSFORM @ np.array([
        [1, 0, 0, -600],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    poe_root = ROOT_TRANSFORM @ np.array([
        [1, 0, 0, 600],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])

    dh_state = dh.State(dh.init_dh, INIT_TRANSFORMS, dh_root, LINK_MESHES)
    poe_state = poe.State(poe.screw_axes_0, INIT_TRANSFORMS, poe_root, LINK_MESHES)
    callbacks = {
        # Press UP to increase a joint angle
        265: lambda _ : dh_state.change_current_angle(np.pi / 32) | poe_state.change_current_angle(np.pi / 32),
        # Press DOWN to decrease a joint angle  
        264: lambda _ : dh_state.change_current_angle(-np.pi / 32) | poe_state.change_current_angle(-np.pi / 32),
        # PressR to show or hide the robot
        ord('R'): lambda _ : dh_state.toggle_robot_display() | poe_state.toggle_robot_display(),
        # F to show or hide the frames
        ord('F'): lambda _ : dh_state.toggle_frame_display() | poe_state.toggle_frame_display(),
    }
    callbacks.update({
        # Press the number keys to switch joint
        ord('1') + i: lambda _, i=i : dh_state.switch_joint(i) | poe_state.switch_joint(i)  for i in range(dh_state.num_joints)
    })

    vis = o3d.visualization.draw_geometries_with_key_callbacks(
        dh_state.meshes.get_meshes() + poe_state.meshes.get_meshes(), callbacks, width=800, height=600
    )