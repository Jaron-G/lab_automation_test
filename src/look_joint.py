import numpy as np
import os
action_path = "/catkin_ws/src/lab_automation_test/action/"
action_list = os.listdir(action_path)
action_list.sort()
for i in action_list:
    if i[-1] == 'M':
        # joint_goal = np.load(action_path+ i + "/current_joint_angle.npy")
        # print("action  " + i + "  complete")
        # print(joint_goal*180/np.pi)
        # print(joint_goal)
        
        print("action  " + i + "  complete")
        position = np.load(action_path + i + "/current_position.npy")
        rotation = np.load(action_path + i + "/current_rotation.npy")
        print(position)
        print(rotation)
        
    


