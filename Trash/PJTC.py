import rospy
import numpy as np
import copy
import time

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

from Kinematics.panda.pandaKinematics import pandaKinematics
from Kinematics.panda import pandaVar
from Utils.Interpolate import *

panda = pandaKinematics()



class PJTC: # position joint trajectory controller
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node("position_joint_trajectory_controller")
        self.action = rospy.resolve_name('~follow_joint_trajectory')

        self.client = SimpleActionClient(self.action, FollowJointTrajectoryAction)
        rospy.loginfo("PJTC: Waiting for '" + self.action + "' action to come up")
        self.client.wait_for_server()

        joint_state_topic = "/franka_state_controller/joint_states"
        rospy.loginfo("PJTC: Waiting for message on topic '" + joint_state_topic + "'")

        self.joint_state = rospy.wait_for_message(joint_state_topic, JointState)

        self.fr3_max_dq = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])  # maximum joint velocity depends on the joint position



    def client_send_goal(self, ts, q_traj, duration):   # is this the best choice? ts and q_traj as input?
        st = time.time()

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        # goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        goal.trajectory.joint_names = self.joint_state.name
        goal.goal_time_tolerance = rospy.Duration.from_sec(duration)

        for i in range(len(q_traj)):
            point = JointTrajectoryPoint()
            point.positions = q_traj[i]
            point.time_from_start = rospy.Duration.from_sec(ts[i])  # time_from_start is relative to trajectory.header.stamp


            goal.trajectory.points.append(point)

        # import pdb;pdb.set_trace()

        self.client.send_goal_and_wait(goal)
        result = self.client.get_result()
        print(time.time() - st)

        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logerr('PJTC: Movement was not successful: ' + {
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
            rospy.loginfo('PJTC: Successfully moved into target pose')
        return result


    def set_joint(self, q_des, duration, force_duration=True):
        q_distance = np.abs(q_des - self.joint_state.position)
        duration_exe = self.get_duration(duration_des=duration, q_distance=q_distance, force_duration=force_duration)

        ts, q_traj = interpolate_q([self.joint_state.position, q_des], duration=duration_exe, visualize=True)
        self.client_send_goal(ts, q_traj, duration_exe)



    def set_cartesian(self, Tb_ed, duration, Tb_ee=None, force_duration=True):
        if Tb_ee is None:
            Tb_ee = panda.fk(self.joint_state.position)[0][-1]
        q_des = panda.ik(Tb_ed, q0=self.joint_state.position)
        q_distance = np.abs(q_des - self.joint_state.position)

        duration_exe = self.get_duration(duration_des=duration, q_distance=q_distance, force_duration=force_duration)


        ts, q_traj = interpolate_T(start_T=Tb_ee, end_T=Tb_ed, duration=duration_exe)

        self.client_send_goal(ts, q_traj, duration_exe)



    def get_duration(self, duration_des, q_distance, force_duration=True):
        duration_base = max(max(q_distance / self.fr3_max_dq), 1) * 3   # 3 sec as default

        if force_duration:
            duration_exe = duration_des
        else:
            if duration_des > duration_base:
                duration_exe = duration_base
            else:
                duration_exe = duration_des
        print("duration_chosen:", duration_exe)
        return duration_exe



if __name__ == "__main__":
    import time
    pjtc = PJTC()
    cur_q = pjtc.joint_state.position
    import time; time.sleep(3)

    ## Joint Space Command
    targ_q = cur_q + 0.1 * np.ones(7)
    print("current_joint:", cur_q)
    print("traget_joint:", targ_q)

    pjtc.set_joint(q_des=targ_q, duration=4)
    exit()

    ## Cartesian Space Command
    # cur_T = panda.fk(cur_q)[0][-1]
    # targ_T = np.eye(4)
    # targ_T[:3, -1] = cur_T[:3, -1] + np.array([0.05, 0.05, 0])
    # targ_T[:3, :3] = cur_T[:3, :3]
    # pjtc.set_cartesian(Tb_ed=targ_T, Tb_ee=cur_T, duration=4)

