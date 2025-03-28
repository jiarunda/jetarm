import numpy as np

from robopal.envs.manipulation_tasks.robot_manipulate import ManipulateEnv
import robopal.commons.transform as trans
from robopal.robots.jetarm_med import JetarmPickAndPlace
from robopal.wrappers import GoalEnvWrapper


class PickAndPlaceEnv(ManipulateEnv):
    name = 'PickAndPlace-v2'

    def __init__(self,
                 robot=JetarmPickAndPlace,
                 render_mode='human',
                 control_freq=20,
                 is_show_camera_in_cv=False,
                 controller='CARTIK',
                 # controller='JNTIMP',
                 is_normalized_action=True,
                 is_randomize_end=False,
                 is_randomize_object=True,
                 is_render_camera_offscreen=False,
                 camera_in_render="frontview",
                 camera_in_window="free",
                 ):
        super().__init__(
            robot=robot,
            render_mode=render_mode,
            control_freq=control_freq,
            is_show_camera_in_cv=is_show_camera_in_cv,
            controller=controller,
            is_normalized_action=is_normalized_action,
            is_randomize_end=is_randomize_end,
            is_randomize_object=is_randomize_object,
            is_render_camera_offscreen=is_render_camera_offscreen,
            camera_in_render=camera_in_render,
            camera_in_window=camera_in_window,
        )

        self.obs_dim = (23,)
        self.goal_dim = (3,)
        self.action_dim = (4,)

        self.max_action = 1
        self.min_action = -1

        self.max_episode_steps = 50

    def _get_obs(self) -> dict:
        """ The observation space is 16-dimensional, with the first 3 dimensions corresponding to the position
        of the block, the next 3 dimensions corresponding to the position of the goal, the next 3 dimensions
        corresponding to the position of the gripper, the next 3 dimensions corresponding to the vector
        between the block and the gripper, and the last dimension corresponding to the current gripper opening.
        """
        obs = np.zeros(self.obs_dim)

        obs[0:3] = (  # gripper position in global coordinates
            end_pos := self.get_site_pos('0_grip_site')
        )
        obs[3:6] = (  # block position in global coordinates
            object_pos := self.get_body_pos('green_block')
        )
        obs[6:9] = (  # Relative block position with respect to gripper position in globla coordinates.
                end_pos - object_pos
        )
        obs[9:12] = (  # block rotation
            trans.mat_2_euler(self.get_body_rotm('green_block'))
        )
        obs[12:15] = (  # gripper linear velocity
            end_vel := self.get_site_xvelp('0_grip_site') * self.dt
        )
        object_velp = self.get_body_xvelp('green_block') * self.dt
        obs[15:18] = (  # velocity with respect to the gripper
                object_velp - end_vel
        )

        obs[18:21] = self.get_body_xvelr('green_block') * self.dt
        obs[21:23] = self.robot.end['arm0'].get_finger_observations()

        return obs.copy()

    def _get_achieved_goal(self) -> np.ndarray:
        return self.get_body_pos('green_block')

    def _get_desired_goal(self) -> np.ndarray:
        return self.get_site_pos('goal_site')

    def _get_info(self) -> dict:
        return {
            'is_success': self._is_success(self.get_body_pos('green_block'), self.get_site_pos('goal_site'), th=0.02)}

    def reset_object(self):
        if self.is_randomize_object:
            random_x_pos, random_y_pos = np.random.uniform([0.35, -0.15], [0.55, 0.15])
            block_pose = np.array([random_x_pos, random_y_pos, 0.46, 1.0, 0.0, 0.0, 0.0])

            goal_pos = np.random.uniform([0.35, -0.15, 0.46], [0.55, 0.15, 0.65])
            while np.linalg.norm(block_pose[:3] - goal_pos) <= 0.05:
                goal_pos = np.random.uniform([0.35, -0.15, 0.46], [0.55, 0.15, 0.65])

            self.set_object_pose('green_block:joint', block_pose)
            self.set_site_pos('goal_site', goal_pos)

        return super().reset_object()


if __name__ == "__main__":
    env = PickAndPlaceEnv(
        is_render_camera_offscreen=True,
        is_randomize_end=False,
        is_randomize_object=False,
    )
    # ============================================================================================================JNTIMP
    # env.controller.B = 0.5 * np.ones(env.robot.jnt_num)  # 20.0 增大B反而有抑制振荡的作用
    # env.controller.K = 0.1 * np.ones(env.robot.jnt_num)  # 80.0
    # ============================================================================================================CARTIK
    env.controller.B = 16.0 * np.ones(env.robot.jnt_num)  # 20.0 设置微分系数
    env.controller.K = 16.0 * np.ones(env.robot.jnt_num)  # 80.0 设置比例系数
    # ==================================================================================================================
    env = GoalEnvWrapper(env)
    env.reset()
    for t in range(int(1e5)):
        action = np.random.uniform(env.min_action, env.max_action, env.action_dim)
        s_, r, terminated, truncated, info = env.step(action)
        if truncated:
            env.reset()
    env.close()
