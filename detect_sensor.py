import copy
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from camera_param import *

class Detect_Sensor():
    def __init__(self):

        ## Variables ##
        image_topic = "/camera/color/image_raw"
        self.distCoeffs = distCoeffs
        self.camMatrix = camMatrix


        ## Utils ##
        self.bridge = CvBridge()

        ## Subscriber ##
        rospy.Subscriber(image_topic, Image, self.image_callback)

        ## Initialize ##
        self.cv2_img = 0
        self.corners = 0
        self.ids = 0
        self.rejected = 0
        self.aruco_visualization = 0
        self.detect_flag = False
        self.num_detected = 0

        rospy.wait_for_message(image_topic, Image)

        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParam = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(arucoDict, arucoParam)


    def image_callback(self, msg):
        self.cv2_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.frame = copy.deepcopy(self.cv2_img)



    def find_aruco(self):
        self.corners, self.ids, self.rejected = self.detector.detectMarkers(self.cv2_img)
        if self.ids is None:
            self.ids = []
        else:
            self.ids = self.ids.ravel() # 1D flatten, ground memory affected

        self.corners = self.corners.reshape(len(self.corners), 4, 2)
        self.corners = self.corners[np.argsort(self.ids)]


    def detect_sensor(self, visualize=False):
        self.find_aruco()

        aruco_poses = np.zeros((4, 6))
        for i in range(len(self.ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(self.corners[i], 0.06, self.camMatrix, self.distCoeffs)
            pos_x, pos_y, pos_z = tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]
            rot_x, rot_y, rot_z = rvec[0][0][0], rvec[0][0][1], rvec[0][0][2]
            # [[quat_x, quat_y, quat_z, quat_w]] = R.from_rotvec(rvec.reshape(1, 3)).as_quat()
            # [roll_x, pitch_y, yaw_z] = R.from_quat(np.array([quat_x, quat_y, quat_z, quat_w])).as_euler('xyz', degrees=True)
            # aruco_poses[i] = [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]
            aruco_poses[i] = [pos_x, pos_y, pos_z, rot_x, rot_y, rot_z]

            if visualize:
                cv2.namedWindow("visualize_detected_aruco", cv2.WINDOW_NORMAL)
                cv2.drawFrameAxes(self.frame, self.camMatrix, self.distCoeffs, rvec, tvec, 0.03, 1)

        if visualize:
            cv2.aruco.drawDetectedMarkers(self.frame, self.corners)
            cv2.imshow("visualize_detected_aruco")

        found_id = np.argwhere(np.sum(aruco_poses, axis=1)!= 0)
        # if len(found_id) >= 3:
        #     sensor_pose =

        return sensor_pose









if __name__ == '__main__':
    rospy.init_node('realsense_aruco_pub')

    da = Detect_Sensor()
    rate = rospy.Rate(300)

    while not rospy.is_shutdown():
        sensor_pose = da.detect_sensor()
        # rate.sleep()
        print(sensor_pose)




