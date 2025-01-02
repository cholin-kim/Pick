from Kinematics.panda import pandaVar as pandaVar
# import pandaVar as pandaVar
from scipy.spatial.transform import Rotation
import scipy.misc
import numpy as np
np.set_printoptions(precision=5, suppress=True)
import time


class pandaKinematics:
    # @classmethod
    # def DH_transform(cls, dh_params):  # stacks transforms of neighbor frame, following the modified DH convention
    #     """
    #     dh_params: N_conf x N_dh x 4
    #     N_conf = # of configurations
    #     N_dh = # of coordinate transforms per DH-param group (9 for Panda: T01, ..., T89)
    #     4 = (alpha, a, d, theta)
    #     """
    #     N_conf, N_dh, _ = np.shape(dh_params)
    #     alpha, a, d, theta = dh_params.reshape(-1, 4).T
    #     Ts = np.zeros((N_conf, N_dh, 4, 4))
    #     Ts[:, :, 0, 0] = np.cos(theta)
    #     Ts[:, :, 0, 1] = -np.sin(theta)
    #     Ts[:, :, 0, 2] = 0.0
    #     Ts[:, :, 0, 3] = a
    #     Ts[:, :, 1, 0] = np.sin(theta) * np.cos(alpha)
    #     Ts[:, :, 1, 1] = np.cos(theta) * np.cos(alpha)
    #     Ts[:, :, 1, 2] = -np.sin(alpha)
    #     Ts[:, :, 1, 3] = -np.sin(alpha) * d
    #     Ts[:, :, 2, 0] = np.sin(theta) * np.sin(alpha)
    #     Ts[:, :, 2, 1] = np.cos(theta) * np.sin(alpha)
    #     Ts[:, :, 2, 2] = np.cos(alpha)
    #     Ts[:, :, 2, 3] = np.cos(alpha) * d
    #     Ts[:, :, 3, 0] = 0.0
    #     Ts[:, :, 3, 1] = 0.0
    #     Ts[:, :, 3, 2] = 0.0
    #     Ts[:, :, 3, 3] = 1.0
    #     return Ts

    @classmethod
    def DH_transform(cls, dhparams):  # stacks transforms of neighbor frame, following the modified DH convention
        Ts = [np.array([[np.cos(theta), -np.sin(theta), 0, a],
                        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),
                         -np.sin(alpha) * d],
                        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
                        [0, 0, 0, 1]]) for [alpha, a, d, theta] in dhparams]
        return Ts

    @classmethod
    def fk(cls, joints):
        """
        joints = (7,)
        Ts = (Tb1, T12, T23, ...)
        """
        Ts = pandaKinematics.DH_transform(pandaVar.dhparam_arm(joints))     # Tb1, T12, T23, ...

        # Calculate forward kin.
        # Never use multi_dot for speed-up. It slows down the computation A LOT! Use "for-loop" instead.
        Tbi = np.eye(4)
        Tbs = []
        for T in Ts:
            # Tbe = Tbe @ T
            Tbi = Tbi.dot(T)
            Tbs.append(Tbi)

        # Tbe = np.linalg.multi_dot(Ts)   # from base to end-effector
        # Tbs = np.array([np.linalg.multi_dot(Ts[:i]) if i > 1 else Ts[0] for i in range(1, len(Ts)+1)])  # Tb1, Tb2, Tb3, ...
        # Tbs[-1]: from base to end effector
        return Tbs, Ts

    @classmethod
    def jacobian(cls, Tbs):
        """
        Tbs: Tb0, Tb1, Tb2, ...
        """
        Tbe = Tbs[-1]
        J = np.zeros((6, 7))
        for i in range(7):
            Zi = Tbs[i][:3, 2]  # vector of actuation axis
            J[3:, i] = Zi  # Jw
            Pin = (Tbe[:3, -1] - Tbs[i][:3, -1])  # pos vector from (i) to (n)
            J[:3, i] = np.cross(Zi, Pin)  # Jv
        return J

    @classmethod
    def ik(cls, Tb_ed, q0=None, RRMC=False, k=0.1):     # inverse kinematics using Newton-Raphson Method
        """
        Tb_ed = transform from base to desired end effector
        q0 = initial configuration for iterative N-R method
        RRMC = Resolved-rate Motion Control
        k = step size (scaling) of cartesian error
        """
        assert Tb_ed.shape == (4, 4)
        st = time.time()
        if q0 is None:
            qk = np.array([0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0])  # initial guess
        else:
            qk = np.array(q0)
        iter = 0
        reached = False
        while not reached:
            result_fk = pandaKinematics.fk(joints=qk)
            Tbs, Ts = result_fk

            # Define Cartesian error
            Tb_ec = Tbs[-1]  # base to current ee
            Tec_ed = np.linalg.inv(Tb_ec).dot(Tb_ed)     #   transform from current ee to desired ee
            pos_err = Tb_ec[:3, :3].dot(Tec_ed[:3, -1])     # pos err in the base frame
            # rot_err = Tb_ec[:3, :3].dot(Rotation.from_dcm(Tec_ed[:3, :3]).as_rotvec())  # rot err in the base frame
            rot_err = Tb_ec[:3, :3].dot(Rotation.from_matrix(Tec_ed[:3, :3]).as_rotvec())  # rot err in the base frame
            err_cart = np.concatenate((pos_err, rot_err))

            # Inverse differential kinematics (Newton-Raphson method)
            J = pandaKinematics.jacobian(Tbs)
            Jp = np.linalg.pinv(J)
            qk_next = qk + Jp.dot(err_cart*k)
            qk = qk_next

            # Convergence condition
            if np.linalg.norm(err_cart) < 10e-4:
                reached = True
            else:
                iter += 1
            if RRMC:
                reached = True

        # print ("iter=", iter, "time=", time.time() - st)
        assert ~np.isnan(qk).any()
        return qk

    @classmethod
    def null_space_control(cls, joints, crit='joint_limit'):    # Null-space control input
        Tbs = pandaKinematics.fk(joints=joints)[0]
        J = pandaKinematics.jacobian(Tbs)
        Jp = np.linalg.pinv(J)
        Jn = np.eye(7) - Jp.dot(J)
        k=0.1
        if crit == 'joint_limit':   # distance to joint limits
            qk_null_dot = [k*pandaKinematics.partial_derivative(pandaKinematics.distance_to_joint_limits, i, joints)
                           for i in range(len(joints))]
        elif crit == 'manipulability':
            qk_null_dot = [k * pandaKinematics.partial_derivative(pandaKinematics.manipulability, i, joints)
                           for i in range(len(joints))]
        elif crit == 'obstacle_avoidance':
            qk_null_dot = [k * pandaKinematics.partial_derivative(pandaKinematics.obstacle_avoidance, i, joints)
                           for i in range(len(joints))]
        else:
            raise ValueError
        return Jn.dot(qk_null_dot)

    @classmethod
    def manipulability(cls, q1, q2, q3, q4, q5, q6, q7):
        J = pandaKinematics.jacobian([q1, q2, q3, q4, q5, q6, q7])
        det = np.linalg.det(J.dot(J.T))
        return np.sqrt(det)

    @classmethod
    def distance_to_joint_limits(cls, q1, q2, q3, q4, q5, q6, q7):
        q = [q1, q2, q3, q4, q5, q6, q7]
        dist = [((q - (q_max+q_min)/2)/(q_max - q_min)) ** 2 for q, q_max, q_min in zip(q, pandaVar.q_max, pandaVar.q_min)]
        return -np.sum(dist)/7/2

    @classmethod
    def obstacle_avoidance(cls, q1, q2, q3, q4, q5, q6, q7):
        q = [q1, q2, q3, q4, q5, q6, q7]
        Tbs = pandaKinematics.fk(joints=q)[0]
        p04 = Tbs[3][:3, -1]   # we can define multiple points on the robot
        p_obj = np.array([0.5, -0.5, 0.3])
        return np.linalg.norm(p04 - p_obj)

    @classmethod
    def partial_derivative(cls, func, var=0, point=[]):
        args = point[:]
        def wraps(x):
            args[var] = x
            return func(*args)
        return scipy.misc.derivative(wraps, point[var], dx=1e-6)


if __name__ == "__main__":
    from panda.pandaRviz import pandaRviz
    panda = pandaRviz(is_core=False)
    while True:
        q_des = (np.random.rand(7) - 0.5) * np.pi/2
        print("q_des=", q_des)
        panda.set_joint_position(joints=np.r_[q_des, 0.0, 0.0])

        # FK
        Tbe_des = pandaKinematics.fk(joints=q_des)[0][-1]
        Tbf = pandaKinematics.fk(joints=q_des)[0][7]
        print ("Tbf=", Tbf)
        # Rotation.from_quat([0.907094, -0.220584, 0.341373, 0.10949]).as_matrix()

        pos_des = Tbe_des[:3, -1]
        quat_des = Rotation.from_matrix(Tbe_des[:3, :3]).as_quat()
        print("pose_des=", pos_des, quat_des)
        # input()

        # IK
        q_ik = pandaKinematics.ik(Tb_ed=Tbe_des, q0=None, RRMC=False, k=0.5)
        panda.set_joint_position(joints=np.r_[q_ik, 0.0, 0.0])
        time.sleep(0.1)
        panda.set_joint_position(joints=np.r_[q_ik, 0.0, 0.0])
        time.sleep(0.1)
        panda.set_joint_position(joints=np.r_[q_ik, 0.0, 0.0])
        time.sleep(0.1)
        panda.set_joint_position(joints=np.r_[q_ik, 0.0, 0.0])
        time.sleep(0.1)
        panda.set_joint_position(joints=np.r_[q_ik, 0.0, 0.0])
        time.sleep(0.1)
        panda.set_joint_position(joints=np.r_[q_ik, 0.0, 0.0])
        print("q_ik =", q_ik)

        # FK to verify the IK
        Tbe_ik = pandaKinematics.fk(joints=q_ik)[0][-1]
        pos_ik = Tbe_ik[:3, -1]
        quat_ik = Rotation.from_matrix(Tbe_ik[:3, :3]).as_quat()
        print("pose_ik=", pos_ik, quat_ik)
        print("======================================================================")
        input()