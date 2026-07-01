import numpy as np


ARX_DEFAULT_ROBOT = "Arx5"
ARX_DEFAULT_GRIPPER = "ArxGripper"
ARX_DEFAULT_BASE = "NullBase"
ARX_TABLE_FULL_SIZE = (1.2, 0.9, 0.05)
ARX_TABLE_OFFSET = np.array((0, 0, 0.8))
ARX_ROBOT_BASE_POS = (-0.535, -0.21, 0.8)
ARX_CAMERA_NAMES = ("external_cam", "robot0_right_eye_in_hand")


def set_arx_base_pose(robot_model):
    robot_model.set_base_xpos(ARX_ROBOT_BASE_POS)


def set_object_pose(sim, obj, pos, quat, use_model_body_pos=False):
    if use_model_body_pos or len(obj.joints) == 0:
        body_id = sim.model.body_name2id(obj.root_body)
        sim.model.body_pos[body_id] = np.array(pos)
        sim.model.body_quat[body_id] = np.array(quat)
    else:
        sim.data.set_joint_qpos(obj.joints[0], np.concatenate([np.array(pos), np.array(quat)]))
