import os, sys
sys.path.append(os.path.abspath("/home/surglab/PegInHole/AutonomousControl"))
import rospy
import copy
import numpy as np
from scipy.spatial.transform import Rotation as R

from AutonomousControl.MoveitCommander import gripper_len
from detect_marker import Detect_Marker
from MoveitCommander import MvitCommander

from Kinematics.panda.pandaKinematics import pandaKinematics
from Camera.eye_in_hand_param import *
from Gripper.DHGripperROS import DHGripperROS

panda = pandaKinematics()
gripper = DHGripperROS()
dm = Detect_Marker(marker_size=0.0125)
mvit = MvitCommander()


if not rospy.get_node_uri():
    rospy.init_node("pickup")

## Temporary code for testing cartesian path
# cur_q = np.array(mvit.group.get_current_joint_values())
# Tb_ed = np.copy(panda.fk(cur_q)[0][-1])
# Tb_ed[2, -1] -= gripper_len
# Tb_ed[1, -1] -= 0.1
# mvit.set_cartesian_path(Tb_ed=Tb_ed, execute=True)
# quit()
#
# # import time; time.sleep(1)
# Tb_ed[1, -1] -= 0.1
# # mvit.set_cartesian_path(Tb_ed=Tb_ed, execute=True)
# # time.sleep(1)
# Tb_ed[2, -1] += 0.1
# # mvit.set_cartesian_path(Tb_ed=Tb_ed, execute=True)
#
# # time.sleep(1)
# Tb_ed[1, -1] += 0.1
# # mvit.set_cartesian_path(Tb_ed=Tb_ed, execute=True)
# quit()


# 0. Open Gripper
rospy.loginfo("Releasing the Gripper.")
gripper.set_gripper(position=1000, speed=gripper.speed, force=20, initialize=False)


# 1. Move to waypoint that can see all possible poses   #<- motion scale joint motion scale
rospy.loginfo("Move to the waypoint for sensor detection.")
## should be manually set depending on the jig location
targ_q1 = np.array([0.9319362832979685, -0.4030470100789721, 0.3298888907845044, -2.08742357272127, 0.06789568625091212, 1.6912255419734388, 2.0481327440708523])
# mvit.set_joint(target_q=targ_q1)
# mvit.set_joint(target_q=targ_q1, execute=True)

Tb_flange1 = panda.fk(targ_q1)[0][-1]
# print(R.from_matrix(Tb_flange1[:3, :3]).as_euler('ZYX'))
Tb_ed1 = copy.deepcopy(Tb_flange1)
Tb_ed1[2, -1] -= gripper_len
Tb_ed1[:3, :3] = R.from_euler('ZX', [-np.pi/4, np.pi]).as_matrix()
# Tb_ed1[:3, :3] = R.from_euler('ZX', [np.pi/2, -np.pi]).as_matrix()
# mvit.set_Tb_ed(Tb_ed=Tb_ed1)
mvit.set_Tb_ed(Tb_ed=Tb_ed1, execute=True)
print("Tb_ed1:", Tb_ed1)



# 2. Detect Aruco
rospy.loginfo("Detecting Sensor Pose.")
detected_ids, aruco_poses = dm.detect_marker(visualize=False)
print("detected_ids:", detected_ids)
print("aruco_poses:", aruco_poses)

Tb_flagne = panda.fk(mvit.group.get_current_joint_values())[0][-1]  # base to link8
Tb_ee = copy.deepcopy(Tb_flagne)
Tb_ee[2, -1] -= gripper_len # base to gripper tcp


def get_sensor_pos(aruco_poses):
    Tb_marker0 = np.identity(4)
    Tb_marker0[:3, -1] = aruco_poses[0][:3]
    Tb_marker0 = Tb_flagne @ Tee_cam @ Tb_marker0

    Tb_marker1 = np.identity(4)
    Tb_marker1[:3, -1] = aruco_poses[1][:3]
    Tb_marker1 = Tb_flagne @ Tee_cam @ Tb_marker1

    Tb_marker2 = np.identity(4)
    Tb_marker2[:3, -1] = aruco_poses[2][:3]
    Tb_marker2 = Tb_flagne @ Tee_cam @ Tb_marker2

    Tb_marker3 = np.identity(4)
    Tb_marker3[:3, -1] = aruco_poses[3][:3]
    Tb_marker3 = Tb_flagne @ Tee_cam @ Tb_marker3

    Tb_sensor = np.identity(4)
    Tb_sensor[:3, -1] = np.mean((Tb_marker0[:3, -1], Tb_marker1[:3, -1], Tb_marker2[:3, -1], Tb_marker3[:3, -1]), axis=0)
    print("Tb_sensor:", Tb_sensor)

    return Tb_sensor    # only position is valid


import random
while not len(detected_ids[0]) == 4:
    Tb_ed = copy.deepcopy(Tb_ee)
    choice = random.choice([0, 1, 2, 3])
    if choice == 0:
        Tb_ed[0, -1] += 0.01
    elif choice == 1:
        Tb_ed[1, -1] += 0.01
    elif choice == 2:
        Tb_ed[0, -1] -= 0.01
    elif choice == 3:
        Tb_ed[1, -1] -= 0.01
    Tb_ee = Tb_ed
    mvit.set_Tb_ed(Tb_ed=Tb_ed, execute=True)
    # mvit.set_Tb_ed(Tb_ed=Tb_ed)

    rospy.sleep(0.5)
    detected_ids, aruco_poses = dm.detect_marker(visualize=False)
    print("detected_ids:", detected_ids)
    print("aruco_poses:", aruco_poses)

Tb_sensor = get_sensor_pos(aruco_poses)
Tb_sensor[:3, :3] = Tb_ed1[:3, :3]


# 3. Move to waypoint(10cm above the marker surface)
rospy.loginfo("Get ready to grasp the sensor.")
Tb_wp = copy.deepcopy(Tb_sensor)
Tb_wp[2, -1] += 0.1
# mvit.set_Tb_ed(Tb_ed=Tb_wp)
mvit.set_Tb_ed(Tb_ed=Tb_wp, execute=True)
print("Tb_wp:", Tb_wp)
# print("after movement:", mvit.group.get_current_pose().pose)



# 4. Open Gripper
rospy.loginfo("Opening the gripper.")
gripper.set_gripper(position=1000, speed=gripper.speed, force=20, initialize=False)


# 5. Go down & Set Gripper Ready
rospy.loginfo("Going down straight and set gripper ready to grasp.")
# gripper.set_gripper(position=800, speed=gripper.speed, force=20, initialize=False)
Tb_ed = copy.deepcopy(Tb_sensor)
Tb_ed[2, -1] += 0.03
print("Tb_ed:", Tb_ed)
# mvit.set_cartesian_path(Tb_ed=Tb_sensor)
mvit.set_cartesian_path(Tb_ed=Tb_ed, execute=True)
gripper.set_gripper_ready()
rospy.sleep(0.5)



# 6. Close Gipper
rospy.loginfo("Going down further and closing the gripper.")
Tb_ed = copy.deepcopy(Tb_ed)
Tb_ed[2, -1] -= 0.005
mvit.set_cartesian_path(Tb_ed=Tb_ed, execute=True)
gripper.set_gripper_grasp()
rospy.sleep(0.5)

# 7. Go up
rospy.loginfo("Sensor grasped, going up.")
targ_pose = copy.deepcopy(mvit.group.get_current_pose().pose)
targ_pose.position.z += 0.1
# mvit.set_pose(pose=targ_pose)
mvit.set_pose(pose=targ_pose, execute=True)



