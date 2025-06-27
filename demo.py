#!/usr/bin/env python

import traceback

"""
Realman robot control demo using the universal controller framework.
This demo shows how to use the framework for basic arm control operations.
"""
import time
import numpy as np
from arm import ArmUnit, RealArmController
from realman import RealmanArmModel
from robotic_arm_package.robotic_arm import RM75  # 导入Realman机械臂型号常量


def main():
    """主演示函数，展示通用控制器框架的基本使用方法"""
    print("启动Realman机械臂控制演示...")

    try:
        # 初始化机械臂模型
        arm_model = RealmanArmModel()
        print(f"已创建机械臂模型: {arm_model.get_model_info()['name']}")

        # 初始化真实机械臂控制器，使用Realman RM75型号
        ip_address1 = "169.254.128.18"  # 请替换为实际的机械臂IP地址
        controller = RealArmController(RM75, ip_address1)
        print(f"尝试连接Realman机械臂，IP: {ip_address1}...")

        # 创建机械臂单元，组合模型和控制器
        arm_unit1 = ArmUnit(controller, arm_model, device_name="left_arm")

        # 初始化机械臂
        if not arm_unit1.initialize():
            print("机械臂初始化失败，请检查连接状态")
            return

        print("机械臂初始化成功！")
        print("执行示例动作序列...")

        # 示例1：移动到关节角度位置
        # print("1. 移动到安全位置...")
        # safe_position = [0, 25, 0, 80, 0, 75, 0]  # 安全的关节角度位置
        # arm_unit1.move_to_joint_position(safe_position)
        # time.sleep(3)  # 等待动作完成

        # 示例2：获取当前位置信息
        joint_positions = arm_unit1.get_joint_positions()
        print(f"当前关节角度: {joint_positions}")

        pos, orient = arm_unit1.get_cartesian_position()
        print(f"当前笛卡尔坐标: 位置={pos}, 方向={orient}")

        # 示例1：移动到关节角度位置
        print(" 移动到安全位置...")
        safe_position = joint_positions  # 安全的关节角度位置
        safe_position[1] += 5
        arm_unit1.move_to_joint_position(safe_position)
        time.sleep(3)  # 等待动作完成

        # # 示例3：移动到笛卡尔空间位置
        # print("2. 执行笛卡尔空间移动...")
        # # 在当前位置基础上微调
        # new_position = [pos[0] + 0.05, pos[1], pos[2] + 0.03]  # 向x+和z+方向移动
        # arm_unit.move_to_cartesian_position(new_position, orient)
        # time.sleep(3)  # 等待动作完成

        # # 示例4：执行多点轨迹
        # print("3. 执行简单轨迹...")
        # execute_simple_trajectory(arm_unit)

        print("演示完成，正在关闭连接...")
        arm_unit1.close()
        print("演示程序结束")

    except Exception as e:
        print(f"发生错误: {e}")
        print("尝试关闭机械臂连接...")
        print(traceback.format_exc(e))
        try:
            if "arm_unit1" in locals():
                arm_unit1.close()
        except:
            pass


def execute_simple_trajectory(arm_unit):
    """执行简单的方形轨迹演示"""
    # 获取当前位置作为起点
    pos, orient = arm_unit.get_cartesian_position()
    start_pos = list(pos)

    # 定义方形轨迹的四个角点（相对于起始位置）
    waypoints = [
        [0.00, 0.00, 0.00],  # 起始点
        [0.10, 0.00, 0.00],  # 向x+移动10cm
        [0.10, 0.10, 0.00],  # 向y+移动10cm
        [0.00, 0.10, 0.00],  # 向x-移动10cm
        [0.00, 0.00, 0.00],  # 回到起始点
    ]

    # 执行轨迹
    for i, wp in enumerate(waypoints):
        # 计算绝对位置
        target = [start_pos[0] + wp[0], start_pos[1] + wp[1], start_pos[2] + wp[2]]
        print(f"  移动到轨迹点 {i+1}/{len(waypoints)}: {target}")

        # 移动到目标点
        try:
            arm_unit.move_to_cartesian_position(target, orient)
            time.sleep(2)  # 等待动作完成
        except ValueError as e:
            print(f"  轨迹点移动失败: {e}")
            # 如果笛卡尔移动失败，可能是因为该点不在工作空间内
            # 这里可以添加应急处理，例如返回关节空间移动


if __name__ == "__main__":
    main()
