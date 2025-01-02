import sys
import copy
from turtledemo.chaos import jumpto

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R
import numpy as np
from Kinematics.panda.pandaKinematics import pandaKinematics
from Kinematics.panda import pandaVar
panda = pandaKinematics()
gripper_len = 0.20  # flange to tcp

class MvitCommander:
    def __init__(self):
        print("============ Starting tutorial setup")
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.get_node_uri():
            rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("fr3_eih_arm")
        self.group.set_max_velocity_scaling_factor(0.7)
        self.group.set_max_acceleration_scaling_factor(0.5)
        # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

        ## Getting Basic Information
        print("============ Reference frame: %s" % self.group.get_planning_frame())
        print("============ Reference frame: %s" % self.group.get_end_effector_link())
        print("============ Robot Groups:")
        print(self.robot.get_group_names())


        self.joint_state = self.group.get_current_joint_values()
        # print("============ Printing robot state")
        # print( self.robot.get_current_state())
        # print( "============")

    def set_Tb_ed(self, Tb_ed, execute=False):
        self.group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        Tb_ed = self.preprocess_T(Tb_ed=Tb_ed)

        target_ori = R.from_matrix(Tb_ed[:3, :3]).as_quat()
        pose_target.orientation.x = target_ori[0]
        pose_target.orientation.y = target_ori[1]
        pose_target.orientation.z = target_ori[2]
        pose_target.orientation.w = target_ori[3]
        pose_target.position.x = Tb_ed[0, -1]
        pose_target.position.y = Tb_ed[1, -1]
        pose_target.position.z = Tb_ed[2, -1]
        self.group.set_pose_target(pose_target)

        plan = self.group.plan()
        rospy.sleep(2)

        # Done automatically by group.plan()
        # print "============ Visualizing plan1"
        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # display_trajectory.trajectory_start = robot.get_current_state()
        # display_trajectory.trajectory.append(plan1)
        # display_trajectory_publisher.publish(display_trajectory);
        # print "============ Waiting while plan1 is visualized (again)..."
        # rospy.sleep(5)

        if execute: self.group.go(wait=True)

    def set_cartesian_path(self, Tb_ed, execute=False):
        self.group.clear_pose_targets()
        waypoints = []
        Tb_ed = self.preprocess_T(Tb_ed=Tb_ed)

        # start with the current pose <-- do not add current pose, this will interrupt duration.
        # waypoints.append(self.group.get_current_pose().pose)

        wpose = geometry_msgs.msg.Pose()
        target_ori = R.from_matrix(Tb_ed[:3, :3]).as_quat()
        wpose.orientation.x = target_ori[0]
        wpose.orientation.y = target_ori[1]
        wpose.orientation.z = target_ori[2]
        wpose.orientation.w = target_ori[3]
        wpose.position.x = Tb_ed[0, -1]
        wpose.position.y = Tb_ed[1, -1]
        wpose.position.z = Tb_ed[2, -1]
        waypoints.append(copy.deepcopy(wpose))

        # print(waypoints)
        planning_step = 0.001  # 0.01 = plan eef with 1cm step, lower than 1mm makes robot unstable
        (plan, fraction) = self.group.compute_cartesian_path(waypoints=waypoints, eef_step=planning_step)

        #########################################################################################
        ## !!!Do not modify duration!!! ##
        # duration = np.linalg.norm([(waypoints[0].position.x - waypoints[-1].position.x), (waypoints[0].position.y - waypoints[-1].position.y), (waypoints[0].position.z - waypoints[-1].position.z)]) / planning_step
        # t = np.linspace(0, duration, len(plan.joint_trajectory.points))
        # for i in range(len(plan.joint_trajectory.points)):
        #     plan.joint_trajectory.points[i].time_from_start = rospy.Duration.from_sec(t[i])
        #########################################################################################

        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.add_subplot(311)
        ax2 = fig.add_subplot(312)
        ax3 = fig.add_subplot(313)
        ax.set_title("joint position for joint4")
        ax2.set_title("joint velocity for joint4")
        ax3.set_title("joint acceleration for joint4")

        for i in range(len(plan.joint_trajectory.points)):
            t = plan.joint_trajectory.points[i].time_from_start.nsecs + plan.joint_trajectory.points[i].time_from_start.secs * 1e9
            q4 = plan.joint_trajectory.points[i].positions[3]
            dq4 = plan.joint_trajectory.points[i].velocities[3]
            ddq4 = plan.joint_trajectory.points[i].accelerations[3]

            ax.scatter(t, q4, c='r')
            ax2.scatter(t, dq4, c='r')
            ax3.scatter(t, ddq4, c='r')
        plt.suptitle("ompl time parameterization: IterativeSplineParameterization")
        plt.show()

        # print(plan)

        rospy.sleep(2)
        if execute: self.group.execute(plan, wait=True)



    def set_joint(self, target_q, execute=False):
        ## Warning
        # joint values should be obtained after considering gripper len!!!
        ##

        self.group.clear_pose_targets()

        group_variable_values = self.group.get_current_joint_values()
        print("============ Joint values: ", group_variable_values)
        group_variable_values = target_q
        self.group.set_joint_value_target(group_variable_values)

        plan = self.group.plan()
        rospy.sleep(2)

        if execute: self.group.go(wait=True)


    def set_pose(self, pose, execute=False):
        '''
        :param pose: geometry_msgs.msg.Pose()
        '''
        pose_target = pose
        self.group.set_pose_target(pose_target)

        plan = self.group.plan()
        rospy.sleep(2)

        if execute: self.group.go(wait=True)

    def preprocess_T(self, Tb_ed):
        Tflagne_tcp = np.identity(4)
        Tflagne_tcp[2, -1] = gripper_len
        Tb_flange = Tb_ed @ np.linalg.inv(Tflagne_tcp)
        return Tb_flange

if __name__ == "__main__":
    mvit = MvitCommander()
    print("cur_q:", mvit.joint_state)
    # target_q = mvit.joint_state
    # target_q += 0.05 * np.ones(7)
    # print("target_q:", target_q)

    # mvit.set_joint(target_q=target_q, execute=True)

    Tb_ed = np.identity(4)
    Tb_ed[:3, :3] = R.from_euler('XZ', [np.pi, -np.pi/2]).as_matrix()
    # Tb_ed1[:3, :3] = R.from_euler('X', [np.pi]).as_matrix()
    Tb_ed[:3, -1] = [0.5, 0.1, 0.5]
    # q = panda.ik(Tb_ed1)
    # mvit.set_joint(target_q = q)

    pose_target = geometry_msgs.msg.Pose()

    target_ori = R.from_matrix(Tb_ed[:3, :3]).as_quat()
    pose_target.orientation.x = target_ori[0]
    pose_target.orientation.y = target_ori[1]
    pose_target.orientation.z = target_ori[2]
    pose_target.orientation.w = target_ori[3]
    pose_target.position.x = Tb_ed[0, -1]
    pose_target.position.y = Tb_ed[1, -1]
    pose_target.position.z = Tb_ed[2, -1]

    pose_target.position.z += gripper_len

    # mvit.group.set_pose_target(pose_target)
    #
    # plan = mvit.group.plan()
    # rospy.sleep(2)
    # mvit.group.go(wait=True)


