import rospy
from dh_gripper_msgs.msg import GripperCtrl

class DHGripperROS:
    def __init__(self):
        # ROS publisher
        self._set_gripper_pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, latch=True, queue_size=1)

        # Initialize ROS node
        if not rospy.get_node_uri():
            rospy.init_node("dh_robotics_gripper_publisher", anonymous=True)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

        self.speed = 100


    def set_gripper(self, position, speed, force, initialize=False):
        """
        :param position: 0 ~ 1000 (permille)  (from spec, 0~10mm stroke per jaw)
        :param speed: 0 ~ 100 (%)  (from spec, 0.2s opening/closing time)
        :param force: 0 ~ 100 (%)  (from spec, 20~80N gripping force per jaw)
        :param initialize: True/False
        :return:
        """
        msg = GripperCtrl()
        msg.initialize = initialize
        msg.position = position
        msg.speed = speed
        msg.force = force
        self._set_gripper_pub.publish(msg)


    '''
    position and the force parameters in the below functions are determined heuristically.
    '''
    def set_gripper_ready(self):
        # ready to grab sensor
        position = 500
        force = 20
        self.set_gripper(position=position, speed=self.speed, force=force, initialize=False)

    def set_gripper_grasp(self):
        # grasp tightly
        position = 200
        force = 30
        self.set_gripper(position=position, speed=self.speed, force=force, initialize=False)


if __name__ == "__main__":
    gripper = DHGripperROS()
    position = [0, 1000]
    speed = 100
    force = 20
    gripper.set_gripper(position=1000, speed=speed, force=force, initialize=False)
    rospy.sleep(2)
    print("start")
    while True:

        # gripper.set_gripper(position=position[0], speed=speed, force=force, initialize=False)
        # rospy.sleep(1.0)
        # gripper.set_gripper(position=position[1], speed=speed, force=force, initialize=False)
        # rospy.sleep(1.0)
        mode = str(input())
        if mode == 'r':
            gripper.set_gripper_ready()
            # rospy.sleep(2.0)
        elif mode == 'g':
            gripper.set_gripper_grasp()
            # rospy.sleep(2.0)
        elif mode == 'q':
            gripper.set_gripper(position=1000, speed=speed, force=force, initialize=False)
            quit()
            # rospy.sleep(2)



