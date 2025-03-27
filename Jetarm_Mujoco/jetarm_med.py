import os

import numpy as np

from robopal.robots.base import BaseRobot

ASSET_DIR = os.path.join(os.path.dirname(__file__), '../assets')


class JetarmMed(BaseRobot):
    """ JetarmMed robot base class. """
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


# class DianaAruco(DianaMed):

#     def add_assets(self):
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/aruco/aruco.xml')
#         # add camera
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/realsense_d435/realsense.xml', parent_body_name='0_link7')


class JetarmCollide(JetarmMed):

    def add_assets(self):
        self.mjcf_generator.add_geom(node='worldbody', name='obstacle_box', pos='0.35 0.0 0.0',
                                     size='0.15 0.01 0.3', type='box')


# class DianaCalib(DianaMed):
#     """ DianaMed for Camera Calibration. """
#     def add_assets(self):
#         # link chessboard to the end
#         self.mjcf_generator.add_texture('chessboard', type='2d',
#                                         file=os.path.join(ASSET_DIR, 'textures/chessboard.png'))
#         self.mjcf_generator.add_material('chessboard', texture='chessboard', texrepeat='1 1', texuniform='false')
#         self.mjcf_generator.add_body(node='0_link7', name='chessboard')
#         self.mjcf_generator.add_geom(node='chessboard', name='chessboard_box', pos='0.0 0 0.0', mass='0.001',
#                                      euler="0 0 1.57", size='0.115 0.08 0.001', type='box', material='chessboard')

#         # set realsense_d435
#         self.mjcf_generator.add_mesh(name="cambase", file="objects/realsense_d435/meshes/cambase.STL")
#         self.mjcf_generator.add_mesh(name="cam", file="objects/realsense_d435/meshes/cam.STL")

#         cam = """<body pos="1.0 0.0 0.8" euler="0 0.785 3.14">
#         <include file="objects/realsense_d435/realsense.xml"/>
#     </body>"""
#         self.mjcf_generator.add_node_from_str('worldbody', cam)


class JetarmGrasp(JetarmMed):
    def __init__(self):
        super().__init__(scene='grasping',
                         gripper='JetarmGripper',
                         mount='top_point')
        self.end_name = {self.agents[0]: '0_eef'}
        self.pos_max_bound = np.array([0.25, 0.25, 0.30])
        self.pos_min_bound = np.array([0.0, -0.25, 0.0])

    def add_assets(self):
        self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/green_cube.xml')

    @property
    def init_qpos(self):
        """ Robot's init joint position. """
        return {self.agents[0]: np.array([0, 0, -1.57, -1.57, 0])}


class JetarmPickAndPlace(JetarmGrasp):

    def add_assets(self):
        super().add_assets()

        goal_site = """<site name="goal_site" pos="0.2 0.0 0.6" size="0.015 0.015 0.015" rgba="1 0 0 1" type="sphere" />"""
        self.mjcf_generator.add_node_from_str('worldbody', goal_site)


# class DianaGraspMultiObjs(DianaPickAndPlace):
#     def add_assets(self):
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/red_cube.xml')
#         self.mjcf_generator.set_node_attrib('body', 'red_block', {'pos': '0.5 -0.1 0.46'})

#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/green_cube.xml')
#         self.mjcf_generator.set_node_attrib('body', 'green_block', {'pos': '0.5 0.0 0.46'})

#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/blue_cube.xml')
#         self.mjcf_generator.set_node_attrib('body', 'blue_block', {'pos': '0.5 0.1 0.46'})

#         r_goal_site = """<site name="red_goal" pos="0.4 0.0 0.5" size="0.015 0.015 0.015" rgba="1 0 0 1" type="sphere" />"""
#         self.mjcf_generator.add_node_from_str('worldbody', r_goal_site)

#         g_goal_site = """<site name="green_goal" pos="0.4 0.0 0.5" size="0.015 0.015 0.015" rgba="0 1 0 1" type="sphere" />"""
#         self.mjcf_generator.add_node_from_str('worldbody', g_goal_site)

#         b_goal_site = """<site name="blue_goal" pos="0.4 0.0 0.5" size="0.015 0.015 0.015" rgba="0 0 1 1" type="sphere" />"""
#         self.mjcf_generator.add_node_from_str('worldbody', b_goal_site)


# class DianaDrawer(DianaPickAndPlace):
#     def add_assets(self):
#         # add cupboard
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cupboard/cupboard.xml')

#     @property
#     def init_qpos(self):
#         """ Robot's init joint position. """
#         return {self.agents[0]: np.array([-0.51198529, -0.44737435, -0.50879166, 2.3063219, 0.46514545, -0.48916244, -0.37233289])}


# class DianaDrawerCube(DianaDrawer):
#     def add_assets(self):
#         super(DianaDrawerCube, self).add_assets()

#         # add cube with random position
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cube/green_cube.xml')
#         self.mjcf_generator.set_node_attrib('body', 'green_block', {'pos': '0.5 0.0 0.46'})

#         goal_site = """<site name="cube_goal" pos="0.59 0.0 0.478" size="0.01 0.01 0.01" rgba="1 0 0 1" type="sphere" />"""
#         self.mjcf_generator.add_node_from_str('worldbody', goal_site)

#     @property
#     def init_qpos(self):
#         """ Robot's init joint position. """
#         return {self.agents[0]: np.array([-0.64551607, -0.29859465, -0.66478589, 2.3211311, 0.3205733, -0.61377277, -0.26366202])}


# class DianaCabinet(DianaPickAndPlace):
#     def add_assets(self):
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cabinet/cabinet.xml')
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/cabinet/beam.xml')

#     @property
#     def init_qpos(self):
#         """ Robot's init joint position. """
#         return {self.agents[0]: np.array([-0.71325374, 0.07279728, -0.72080385, 2.5239552, -0.07686951, -0.67930021, 0.05372948])}


# class DianaAssemble(DianaMed):
#     def __init__(self):
#         super().__init__(scene='grasping',
#                          gripper='RethinkGripper',
#                          mount='top_point')

#     def add_assets(self):
#         self.mjcf_generator.add_node_from_xml(ASSET_DIR + '/objects/motor/motor.xml')
#         for i in range(63):
#             self.mjcf_generator.add_mesh(f'decomp{i}', ASSET_DIR + f'/objects/motor/output/decomp{i}.obj', scale='0.001 0.001 0.001')
#             self.mjcf_generator.add_geom(node='cover', mesh=f'decomp{i}', type='mesh', group='1', friction="0.3 0.005 0.001")

#     @property
#     def init_qpos(self):
#         """ Robot's init joint position. """
#         return {self.agents[0]: np.array([-0.71325374, 0.07279728, -0.72080385, 2.5239552, -0.07686951, -0.67930021, 0.05372948])}
