import rospy
import copy


from scipy.spatial.transform import Rotation as R

from detect_marker import Detect_Marker
from PJTC import PJTC
dm = Detect_Marker(marker_size=0.017)
pjtc = PJTC()

from Kinematics.panda.pandaKinematics import pandaKinematics
from Camera.eye_in_hand_param import *

panda = pandaKinematics()

class Pickup:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node("position_joint_trajectory_controller")

        # rectangle information defined by aruco markers as 4 corners
        # self.w = 0.07
        # self.h = 0.07

        detected_ids, aruco_poses = dm.detect_marker()
        Tcam_sensor = self.get_sensor_pose(detected_ids=detected_ids, aruco_poses=aruco_poses)
        self.ready_to_pick(Tcam_sensor)



    def get_sensor_pose(self, detected_ids, aruco_poses):
        '''
        :param detected_ids:
        :param aruco_poses:
        :return: Tcam_sensor
        '''
        # if len(self.detected_ids) == 1:
        if detected_ids[0] == 0:
            sensor_pose = copy.deepcopy(aruco_poses[0])
            # print("aruco_pose:", sensor_pose)

        ## Additional algorithm will be added soon for multiple markers present in image.
        # # position
        # # sensor_pos[2] = np.mean(aruco_poses[found_id, 2])
        # center_pos_cand = []
        # for id in found_id:
        #     center_pos_cand.append(self.center_cand(aruco_poses[id][:3]))
        # center_pos = np.mean(np.array(center_pos_cand), axis=0)
        #
        # # orientation

        Tcam_sensor = np.eye(4)
        Tcam_sensor[:3, -1] = sensor_pose[:3]
        Tcam_sensor[:3, :3] = R.from_rotvec(sensor_pose[3:]).as_matrix()
        # print("Tcam_sensor:", Tcam_sensor)

        return Tcam_sensor



    def ready_to_pick(self, Tcam_sensor):
        Tb_wp = self.get_waypoint(Tcam_sensor)
        print("Tb_wp:\n", Tb_wp)
        q_wp = panda.ik(Tb_wp)

        # import time; time.sleep(3)
        # pjtc.set_cartesian(Tb_ed=Tb_wp, duration=5, force_duration=True)
        pjtc.set_joint(q_des=q_wp, duration=5, force_duration=True)


    def get_waypoint(self, Tcam_sensor):
        Tb_ee = panda.fk(pjtc.joint_state.position)[0][-1]
        # import pdb;pdb.set_trace()
        Tb_sensor = Tb_ee @ Tee_cam @ Tcam_sensor   #Tee_cam from Camera/eye_in_hand_param.py
        Tb_wp = np.copy(Tb_sensor)
        Tb_wp[2, -1] += 0.3

        # print(R.from_matrix(Tb_wp[:3, :3]).as_euler('XYZ', degrees=True))
        Tb_wp[:3, :3] = R.from_euler('X', [np.pi]).as_matrix()  # temporary
        return Tb_wp

    def ready_to_detect(self):  # manually set
        Tb_rd = np.identity(4)
        Tb_rd[:3, :3] = R.from_euler('X', np.pi).as_matrix()
        Tb_rd[:3, -1] = [0.5, 0, 0.5]











if __name__ == "__main__":
    pu = Pickup()








# def center_cand(self, pos, id):
#     if (id % 4) == 0:
#         return np.array([pos[0] + self.w / 2, pos[1] - self.w / 2, pos[2]])
#     elif (id % 4) == 1:
#         return np.array([pos[0] - self.w / 2, pos[1] - self.w / 2, pos[2]])
#     elif (id % 4) == 2:
#         return np.array([pos[0] + self.w / 2, pos[1] + self.w / 2, pos[2]])
#     elif (id % 4) == 3:
#         return np.array([pos[0] - self.w / 2, pos[1] + self.w / 2, pos[2]])
