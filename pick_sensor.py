import rospy
import sys

from detect_sensor import Detect_Sensor
da = Detect_Sensor()

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult


rospy.init_node("pick_sensor")
action = rospy.resolve_name('~follow_joint_trajectory')
client = SimpleActionClient(action, FollowJointTrajectoryAction)
rospy.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
client.wait_for_server()

param = rospy.resolve_name('~joint_pose')
pose = rospy.get_param(param, None)
if pose is None:
    rospy.logerr('move_to_start: Could not find required parameter "' + param + '"')
    sys.exit(1)

topic = rospy.resolve_name('~joint_states')
rospy.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
joint_state = rospy.wait_for_message(topic, JointState)
initial_pose = dict(zip(joint_state.name, joint_state.position))
# target_pose =

max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

point = JointTrajectoryPoint()
point.time_from_start = rospy.Duration.from_sec(
    # Use either the time to move the furthest joint with 'max_dq' or 500ms,
    # whatever is greater
    max(max_movement / rospy.get_param('~max_dq', 0.5), 0.5)
)
goal = FollowJointTrajectoryGoal()

goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
point.velocities = [0] * len(pose)

goal.trajectory.points.append(point)
goal.goal_time_tolerance = rospy.Duration.from_sec(0.5)

rospy.loginfo('Sending trajectory Goal to move into initial config')
client.send_goal_and_wait(goal)

result = client.get_result()
if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
    rospy.logerr('move_to_start: Movement was not successful: ' + {
        FollowJointTrajectoryResult.INVALID_GOAL:
        """
        The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
        Is the 'joint_pose' reachable?
        """,

        FollowJointTrajectoryResult.INVALID_JOINTS:
        """
        The joint pose you specified is for different joints than the joint trajectory controller
        is claiming. Does you 'joint_pose' include all 7 joints of the robot?
        """,

        FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        """
        During the motion the robot deviated from the planned path too much. Is something blocking
        the robot?
        """,

        FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
        """
        After the motion the robot deviated from the desired goal pose too much. Probably the robot
        didn't reach the joint_pose properly
        """,
    }[result.error_code])

else:
    rospy.loginfo('move_to_start: Successfully moved into start pose')