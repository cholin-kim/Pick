import copy
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from Camera.eye_in_hand_param import *

'''
4X4_50 dictionary set as default.
If you are using other aruco markers or other shape(3X3, ...), you should add aruco dictionary.
e.g., cv2.aruco.DICT_4X4_100
'''

class Detect_Marker:
    def __init__(self, marker_size):
        if not rospy.get_node_uri():
            rospy.init_node("position_joint_trajectory_controller")

        ## Variables ##
        image_topic = "/camera/color/image_raw"
        self.distortion_params = D
        self.intrinsic_matrix = K
        self.marker_size = marker_size


        ## Utils ##
        self.bridge = CvBridge()

        ## Subscriber ##
        rospy.Subscriber(image_topic, Image, self.image_callback)

        ## Initialize ##
        self.cv2_img = 0
        self.corners = 0
        self.ids = 0
        self.num_detected = 0

        rospy.wait_for_message(image_topic, Image)

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParam = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(arucoDict, arucoParam)


    def image_callback(self, msg):
        # self.cv2_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.cv2_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        self.frame = copy.deepcopy(self.cv2_img)



    def find_aruco(self):
        self.corners, self.ids, _ = self.detector.detectMarkers(self.cv2_img)
        # import pdb;pdb.set_trace()
        if self.ids is None:
            self.ids = []
        else:
            self.ids = self.ids.ravel() # 1D flatten, ground memory affected

        self.corners = np.array(self.corners)
        self.corners = self.corners.reshape(len(self.corners), 4, 2)
        self.corners = self.corners[np.argsort(self.ids)]



    def detect_marker(self, visualize=False):
        self.find_aruco()
        aruco_poses = np.zeros((4, 6))

        for i in range(len(self.ids)):
            marker_points = np.array([[-self.marker_size / 2, self.marker_size / 2, 0],
                                      [self.marker_size / 2, self.marker_size / 2, 0],
                                      [self.marker_size / 2, -self.marker_size / 2, 0],
                                      [-self.marker_size / 2, -self.marker_size / 2, 0]], dtype=np.float32)

            _, rvec, tvec = cv2.solvePnP(marker_points, self.corners[i], self.intrinsic_matrix, self.distortion_params)
            # _, rvec, tvec = cv2.solvePnP(marker_points, self.corners[i], self.intrinsic_matrix, self.distortion_params, False, cv2.SOLVEPNP_IPPE_SQUARE)
            pos_x, pos_y, pos_z = tvec[0][0], tvec[1][0], tvec[2][0]
            rot_x, rot_y, rot_z = rvec[0][0], rvec[1][0], rvec[2][0]

            '''
            ## depending on cv version, 
            works for:
             opencv-python-4.9.0.80
             opencv-contrib-python-4.9.0.80
            # rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(self.corners[i], 0.0017, self.intrinsic_matrix, self.distortion_params)  # 0.06: marker length
            # pos_x, pos_y, pos_z = tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]
            # rot_x, rot_y, rot_z = rvec[0][0][0], rvec[0][0][1], rvec[0][0][2] 
            '''

            # [[quat_x, quat_y, quat_z, quat_w]] = R.from_rotvec(rvec.reshape(1, 3)).as_quat()
            # [roll_x, pitch_y, yaw_z] = R.from_quat(np.array([quat_x, quat_y, quat_z, quat_w])).as_euler('xyz', degrees=True)
            aruco_poses[i] = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]

            if visualize:
                cv2.namedWindow("visualize_detected_aruco", cv2.WINDOW_NORMAL)
                cv2.drawFrameAxes(self.frame, self.intrinsic_matrix, self.distortion_params, rvec, tvec, 0.03, 1)

        if visualize:
            # cv2.aruco.drawDetectedMarkers(self.frame, self.corners) # works for opencv-python-4.9.0.80
            cv2.imshow("visualize_detected_aruco", self.frame)
            cv2.waitKey(0)

        found_id = np.where((np.sum(aruco_poses, axis=1) != 0).ravel())

        # return self.ids, aruco_poses
        return found_id, aruco_poses



if __name__ == '__main__':
    rospy.init_node('realsense_aruco_pub')
    from scipy.spatial.transform import Rotation as R

    # da = Detect_Marker(marker_size=0.017)
    da = Detect_Marker(marker_size=0.015)
    rate = rospy.Rate(300)
    while True:
        ids, aruco_poses = da.detect_marker(visualize=True)
        print(ids, aruco_poses)
        import pdb;pdb.set_trace()

        # print(ids)
        # Tcam_aruco = np.eye(4)
        # Tcam_aruco[:3, -1] = aruco_poses[0][:3]
        # Tcam_aruco[:3, :3] = R.from_rotvec(aruco_poses[0][3:]).as_matrix()
        # print(Tcam_aruco)
        # import pdb;pdb.set_trace()
        import time;time.sleep(1)






