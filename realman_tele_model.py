import time

# from type import List
import numpy as np
from arm import ArmUnit, RealArmController
from realman import RealmanArmModel
from robotic_arm_package.robotic_arm import RM75  # 导入Realman机械臂型号常量


class ARM:
    def __init__(self, ip="169.254.128.18", name="left_arm"):
        arm_model = RealmanArmModel()
        print(f"已创建机械臂模型: {arm_model.get_model_info()['name']}")

        controller = RealArmController(RM75, ip)
        print(f"尝试连接Realman机械臂，IP: {ip}...")

        self.arm_unit = ArmUnit(controller, arm_model, device_name=name)

        if not self.arm_unit.initialize():
            print("机械臂初始化失败，请检查连接状态")
            return

    def get_joint_positions(self):
        return self.arm_unit.get_joint_positions()

    def move_to_joint_position(self, positions) -> bool:
        return self.arm_unit.move_to_joint_position(positions)


class RTM:
    def __init__(self):
        self.left_arm = ARM("169.254.128.18", "left_arm")
        self.right_arm = ARM("169.254.128.19", "right_arm")

    def get_joint_positions(self):
        l_joint_positions = self.left_arm.get_joint_positions()
        r_joint_positions = self.right_arm.get_joint_positions()

        return {"left_arm": l_joint_positions, "right_arm": r_joint_positions}

    def mirror_arms(self):
        l_joint_positions = self.left_arm.get_joint_positions()

        r_joint_positions = np.array(l_joint_positions) * np.array(
            [1, -1, -1, -1, 1, 1, 1]
        )
        print(
            "now position is ",
            l_joint_positions,
            "right ",
            self.right_arm.get_joint_positions(),
        )
        input(f"target right {r_joint_positions} press enter to continue")
        self.right_arm.move_to_joint_position(r_joint_positions)
        print("arms mirrored ")


def main():
    """主演示函数，展示通用控制器框架的基本使用方法"""
    print("启动Realman机械臂控制演示...")
    rtm = RTM()
    print(rtm.get_joint_positions())
    rtm.mirror_arms()
    time.sleep(3)


if __name__ == "__main__":
    main()
