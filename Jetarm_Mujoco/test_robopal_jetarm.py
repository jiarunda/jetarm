import numpy as np
import logging

from robopal.robots.jetarm import Jetarm
from robopal.envs import RobotEnv
from robopal.controllers.planners.rrt import rrt_star

logging.basicConfig(level=logging.INFO)

if __name__ == "__main__":

    # options = {'manipulator': Jetarm, 'ctrl': 'CARTIK'}
    options = {'manipulator': Jetarm, 'ctrl': 'JNTIMP'}

    # Initialize the environment
    env = RobotEnv(
        robot=options['manipulator'],
        render_mode='human',
        control_freq=200,
        is_interpolate=False,
        controller=options['ctrl'],
    )

    env.controller.B = 0.5 * np.ones(env.robot.jnt_num)  # 20.0 增大B反而有抑制振荡的作用
    env.controller.K = 0.1 * np.ones(env.robot.jnt_num)  # 80.0

    initial_pos = env.robot.get_end_xpos()
    initial_quat = env.robot.get_end_xquat()
    # ==================================================================================================================
    action = np.random.uniform(low=env.robot.mani_joint_bounds[env.robot.agents[0]][0],
                               high=env.robot.mani_joint_bounds[env.robot.agents[0]][1],
                               size=env.robot.jnt_num)
    # action = np.array([-1.33416692, 0.94138172, -0.2653751, -1.08260181, 1.20947979])
    # ==================================================================================================================
    # action = np.array([0, 0, -1.57, -1.57, 0])
    # action = np.array([0, 0, 0, 0, 0])

    print("Action = ", action)

    # Main loop
    env.reset()
    for _ in range(int(5e3)):
        env.step(action)

    final_pos = env.robot.get_end_xpos()
    final_quat = env.robot.get_end_xquat()
    print("InitialPos = ", initial_pos, "FinalPos = ", final_pos)
    print("InitialQuat = ", initial_quat, "FinalQuat = ", final_quat)

    env.close()

