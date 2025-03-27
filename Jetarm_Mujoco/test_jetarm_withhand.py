import os
import numpy as np
import logging

from robopal.robots.jetarm import JetarmGrasp
from robopal.envs import RobotEnv

ASSET_DIR = os.path.join(os.path.dirname(__file__), '../assets')

if __name__ == "__main__":

    options = {'ctrl': 'CARTIK'}
    # options = {'ctrl': 'JNTIMP'}

    pos = np.zeros(3)

    env = RobotEnv(
        robot=JetarmGrasp,
        render_mode='human',
        control_freq=100,
        controller=options['ctrl'],
    )

    env.controller.B = 16.0 * np.ones(env.robot.jnt_num)  # 20.0 设置微分系数
    env.controller.K = 16.0 * np.ones(env.robot.jnt_num)  # 80.0 设置比例系数

    # env.renderer.camera_in_window = "0_cam"

    env.reset()
    env.controller.reference = 'world'
    env.robot.end['arm0'].open()

    initial_pos = env.robot.get_end_xpos()
    initial_quat = env.robot.get_end_xquat()

    for _ in range(int(100)):
        env.step(initial_pos)

    # ===================================================================================================移动红木块到新位置
    action = env.get_body_pos('red_block')
    action[2] = 0.46  # 校正高度
    print("Action = ", action)
    for _ in range(int(300)):
        env.step(action)
    for t in range(int(150)):
        env.robot.end['arm0'].close()
        env.step(action)
    action += np.array([0.0, 0.0, 0.10])
    print("Action = ", action)
    for t in range(int(300)):
        env.step(action)
    action = np.array([0.15, 0, 0.465])  # 移动到新位置
    print("Action = ", action)
    for t in range(int(500)):
        env.step(action)
    env.robot.end['arm0'].open()
    for t in range(int(200)):
        env.step(action)
    # ===================================================================================================移动绿木块到新位置
    action = env.get_body_pos('green_block')
    action[2] = 0.46  # 校正高度
    print("Action = ", action)
    for _ in range(int(300)):
        env.step(action)
    for t in range(int(150)):
        env.robot.end['arm0'].close()
        env.step(action)
    action += np.array([0.0, 0.0, 0.10])
    print("Action = ", action)
    for t in range(int(300)):
        env.step(action)
    action = np.array([0.15, 0, 0.5])  # 移动到新位置
    print("Action = ", action)
    for t in range(int(500)):
        env.step(action)
    env.robot.end['arm0'].open()
    for t in range(int(200)):
        env.step(action)
    # ==================================================================================================================

    for _ in range(int(300)):
        env.step(initial_pos)
    env.renderer.close_render_window()
    # final_pos = env.robot.get_end_xpos()
    # final_quat = env.robot.get_end_xquat()
    # print("InitialPos = ", initial_pos, "FinalPos = ", final_pos)
    # print("InitialQuat = ", initial_quat, "FinalQuat = ", final_quat)
    env.close()
