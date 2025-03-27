# jetarm
Mujoco code for jetarm robotic arm
1. Install Python 3.11, Mujoco 3.1.5, robopal, pettingzoo, h5py, Set the environment name jetarm
2. Copy the folder JetarmGripper into D:\anaconda3\envs\jetarm\Lib\site-packages\robopal\assets\models\grippers for storing grippers
3. Copy the folder Jetarm into D:\anaconda3\envs\jetarm\Lib\site-packages\robotal\assets\models\manipulators, which are used to store robotic arms
4. Overwrite the cube folder in D:\anaconda3\envs\jetarm\Lib\site-packages\robopal\assets\objects with the cube folder to change the size and friction of the red and green bricks
5. Overwrite the file with the same name in D:\anaconda3\envs\jetarm\Lib\site-packages\robopal\assets\scenes with a grassping.xml file to move the position of the table
6. The robot.xml generated in the D:\anaconda3\envs\jetarm\Lib\site-packages\robopal\assets folder can be viewed in Mujoco
7. The robot.py file in the D:\anaconda3\envs\jetarm\Lib\site-packages\robopal\envs folder can be used to change the camera's perspective. "0_cam" indicates that the camera on the robotic arm is being used
8. Place jetarm.py and jetarm_med.py into the folder D:\anaconda3\envs\jetarm\Lib\site-packages\robopal\robots, set up usage scenarios for robotic arms to achieve tasks such as grasping and path planning
9. Overwrite the gripper.py file with the same name in the folder D:\anaconda3\envs\jetarm\Lib\site-packages\robotal\robots, and register Jetarm's gripper
10. Create a new Python project, test_robopal_jetarm.py, which can detect the effectiveness of the controller, can change the parameters of the controller to reduce oscillation and overshoot
12. The file test_jetar_withhand.py can be used to stack two bricks. If it cannot be clamped, the parameters in the XML files of the bricks and claws can be adjusted  
13. jetarm_pick_place.py can be used for RL learning in the task space