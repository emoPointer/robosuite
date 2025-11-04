import numpy as np
from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion


class ArxGripper(GripperModel):
    """
    UMIGripper for the Arx5 arm
    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("grippers/arx_gripper.xml"), idn=idn)

    def format_action(self, action):
        assert len(action) == self.dof
        # 夹爪有2个关节，但只接受1个控制命令
        # 将1个命令复制到2个关节（镜像运动）
        self.current_action = np.clip(
            self.current_action + np.array([1.0, 1.0]) * self.speed * np.sign(action), 
            -10.0, 10.0
        )
        return self.current_action

    @property
    def init_qpos(self):
        # 两个手指的初始位置
        return np.array([0.01, 0.01])

    @property
    def speed(self):
        return 0.2

    @property
    def _important_geoms(self):
        return {
            "right_fingerpad": ["right_finger_collision", "right_finger_visual"],
            "left_fingerpad": ["left_finger_visual", "left_finger_collision"],
        }
    
    @property
    def dof(self):
        return 1