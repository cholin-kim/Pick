import rospy
from copy import deepcopy
import actionlib
from sensor_msgs.msg import JointState
from franka_gripper.msg import (GraspAction, GraspGoal,
                                HomingAction, HomingGoal,
                                MoveAction, MoveGoal,
                                StopAction, StopGoal,
                                GraspEpsilon)


class pandaGripper:
    def __init__(self, gripper_joint_names=('panda_finger_joint1', 'panda_finger_joint2'), calibrate=False):

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('pandaGripper_node', anonymous=True, log_level=rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

        # To Get Gripper States
        self._joint_positions = dict()
        self._joint_names = gripper_joint_names
        self._joint_velocity = dict()
        self._joint_effort = dict()
        self._joint_states_state_sub = rospy.Subscriber('/franka_gripper/joint_states', JointState, self._joint_states_callback, queue_size=1, tcp_nodelay=True)

        # To Set Gripper Action
        self._gripper_speed = 0.05  # (m/s)
        self._homing_action_client = actionlib.SimpleActionClient("franka_gripper/homing", HomingAction)
        self._grasp_action_client = actionlib.SimpleActionClient("franka_gripper/grasp", GraspAction)
        self._move_action_client = actionlib.SimpleActionClient("franka_gripper/move", MoveAction)
        self._stop_action_client = actionlib.SimpleActionClient("franka_gripper/stop", StopAction)

        rospy.loginfo("GripperInterface: Waiting for gripper action servers... ")
        self._homing_action_client.wait_for_server()
        self._grasp_action_client.wait_for_server()
        self._move_action_client.wait_for_server()
        self._stop_action_client.wait_for_server()
        rospy.loginfo("GripperInterface: Gripper action servers found! ")

        self.MIN_FORCE = 0.01   # (N)
        self.MAX_FORCE = 50     # documentation says upto 70N is possible as continuous force (max upto 140N)
        self.MIN_WIDTH = 0.0001 # (m)
        self.MAX_WIDTH = 0.2    # (m)

        if calibrate:
            self.home(wait_for_result=True)

    """
    Get Gripper Status
    """
    def set_velocity(self, value):
        assert self.MIN_WIDTH <= value <= self.MAX_WIDTH, "GripperInterface: Invalid speed request for gripper joints. Should be within {} and {}.".format(
            self.MIN_WIDTH, self.MAX_WIDTH)
        self._gripper_speed = value

    def _joint_states_callback(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
                self._joint_positions[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    def joint_names(self):
        return self._joint_names

    def joint_positions(self):
        return deepcopy(self._joint_positions)

    def joint_velocities(self):
        return deepcopy(self._joint_velocity)

    def joint_efforts(self):
        return deepcopy(self._joint_effort)

    """
    Set Gripper Action (ROS Actionlib)
    """
    def _active_cb(self):
        rospy.logdebug("pandaGripper: '{}' request active.".format(self._caller))
        print ("pandaGripper: '{}' request active.".format(self._caller))

    def _feedback_cb(self, msg):
        rospy.logdebug("pandaGripper: '{}' request feedback: \n\t{}".format(self._caller,msg))
        print ("pandaGripper: '{}' request feedback: \n\t{}".format(self._caller,msg))

    def _done_cb(self, status, result):
        rospy.logdebug("pandaGripper: '{}' complete. Result: \n\t{}".format(self._caller, result))
        print ("pandaGripper: '{}' complete. Result: \n\t{}".format(self._caller, result))

    def home(self, wait_for_result=False):
        self._caller = "home_gripper"
        goal = HomingGoal()
        self._homing_action_client.send_goal(goal, done_cb=self._done_cb, active_cb=self._active_cb, feedback_cb=self._feedback_cb)
        if wait_for_result:
            result = self._homing_action_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def move(self, width, speed=None, wait_for_result=True):
        self._caller = "move_gripper"

        goal = MoveGoal()
        if not speed:
            speed = self._gripper_speed
        goal.width = width
        goal.speed = speed
        self._move_action_client.send_goal(goal, done_cb=self._done_cb, active_cb=self._active_cb, feedback_cb=self._feedback_cb)
        if wait_for_result:
            result = self._move_action_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def stop(self):
        self._caller = "stop_gripper"
        goal = StopGoal()
        self._stop_action_client.send_goal(goal, done_cb=self._done_cb, active_cb=self._active_cb, feedback_cb=self._feedback_cb)
        result = self._stop_action_client.wait_for_result(rospy.Duration(10))
        return result

    def grasp(self, width, force, speed=None, epsilon_inner=0.005, epsilon_outer=0.005, wait_for_result=False, cb=None):
        """
        Grasps an object.

        An object is considered grasped if the distance :math:`d` between the gripper fingers satisfies
        :math:`(width - epsilon\_inner) < d < (width + epsilon\_outer)`.

        :param width: Size of the object to grasp. [m]
        :param speed: Closing speed. [m/s]
        :param force: Grasping force. [N]
        :param epsilon_inner: Maximum tolerated deviation when the actual grasped width is smaller
                                than the commanded grasp width.
        :param epsilon_outer: Maximum tolerated deviation when the actual grasped width is wider
                                than the commanded grasp width.
        :param cb: Optional callback function to use when the service call is done

        :type width: float
        :type speed: float
        :type force: float
        :type epsilon_inner: float
        :type epsilon_outer: float

        :return: True if an object has been grasped, false otherwise.
        :rtype: bool
        """
        self._caller = "grasp_action"
        if not speed:
            speed = self._gripper_speed
        goal = GraspGoal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon = GraspEpsilon(inner=epsilon_inner, outer=epsilon_outer)
        if not cb:
            cb = self._done_cb
        self._grasp_action_client.send_goal(goal, done_cb=cb, active_cb=self._active_cb, feedback_cb=self._feedback_cb)
        if wait_for_result:
            result = self._grasp_action_client.wait_for_result(rospy.Duration(10))
            return result
        return True

    def open(self, wait_for_result=True):
        self._caller = "open gripper"
        return self.move(self.MAX_WIDTH, wait_for_result=wait_for_result)

    def close(self, wait_for_result=True):
        self._caller = "close gripper"
        return self.grasp(width=0.0, force=0.1, wait_for_result=wait_for_result)


if __name__ == "__main__":
    gripper1 = pandaGripper(calibrate=False)
    gripper1.open(wait_for_result=True)
    gripper1.close(wait_for_result=True)