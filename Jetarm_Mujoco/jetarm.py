import os

import numpy as np
from robopal.robots.base import BaseRobot

ASSET_DIR = os.path.join(os.path.dirname(__file__), '../assets')


class Jetarm(BaseRobot):
    """ Jetarm robot base class. """
    def __init__(self,
                 scene='default',
                 manipulator='Jetarm',
                 gripper=None,
                 mount=None
                 ):
        super().__init__(
            scene=scene,
            mount=mount,
            manipulator=manipulator,
            gripper=gripper,
            attached_body='0_attachment',
        )
        self.arm_joint_names = {self.agents[0]: ['0_joint1', '0_joint2', '0_joint3', '0_joint4', '0_joint5']}
        self.arm_actuator_names = {self.agents[0]: ['0_actuator1', '0_actuator2', '0_actuator3', '0_actuator4', '0_actuator5']}
        self.base_link_name = {self.agents[0]: '0_link0'}
        self.end_name = {self.agents[0]: '0_attachment'}

        self.pos_max_bound = np.array([0.3, 0.3, 0.41])
        self.pos_min_bound = np.array([-0.16, -0.3, 0.0])

    @property
    def init_qpos(self):
        """ Robot's init joint position. """
        return {self.agents[0]: np.array([0, 0, -1.57, -1.57, 0])}
        


class JetarmGrasp(Jetarm):
    def __init__(self):
        super().__init__(scene='grasping',
                         gripper='JetarmGripper',
                         # gripper='PandaHand',
                         # gripper='RobotiqGripper',
                         mount='top_point')

        self.end_name = {self.agents[0]: '0_eef'}

    def add_assets(self):
        self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/green_cube.xml')
        self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/red_cube.xml')
        
    @property
    def init_qpos(self):
        """ Robot's init joint position. """
        return {self.agents[0]: np.array([0, 0, -1.57, -1.57, 0])}
    

class JetarmPickAndPlace(JetarmGrasp):

    def add_assets(self):
        super().add_assets()
        goal_site = """<site name="goal_site" pos="0.4 0.0 0.5" size="0.02 0.02 0.02" rgba="1 0 0 1" type="sphere" />"""
        self.mjcf_generator.add_node_from_str('worldbody', goal_site)
