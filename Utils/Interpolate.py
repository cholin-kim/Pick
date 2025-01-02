import numpy as np

from Kinematics.panda.pandaKinematics import pandaKinematics
panda = pandaKinematics()
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RotationSpline
from scipy.spatial.transform import Slerp


def interpolate_q(q_lst, duration, visualize=False):
    '''
    performs cubic spline interpolation.
    limitation: do not allow us to specify the initial or final acceleration
    assumption: each point in q_lst has same time distance
    :param q_lst: waypoints of trajectory
    :return:
    '''

    t = np.linspace(0, duration, len(q_lst))

    # Too many sampling does not allow robot reaching its interpolated waypoints before the next goal, hence worse performance
    ts = np.linspace(0, duration, duration * 100)

    q_lst = np.array(q_lst).reshape(len(q_lst), -1)

    cs = []
    q_traj = []

    for i in range(len(q_lst[0])):
        cs.append(CubicSpline(t, q_lst[:, i], bc_type='clamped'))

        q_traj_individual = cs[i](ts)
        q_traj.append(q_traj_individual)

    q_traj = np.array(q_traj).T


    if visualize:
        import matplotlib.pyplot as plt
        for i in range(len(q_lst[0])):
            plt.scatter(ts, q_traj[:, i], s=2, label=f'{i}')
        for i in range(len(t)):
            plt.scatter(np.ones(len(q_lst[0])) * t[i], q_lst[i], s=20, c='red')

        plt.legend()
        plt.show()
    return ts, q_traj


def interpolate_T(start_T, end_T, duration, visualize=False):
    t = np.array([0, duration])
    ts = np.linspace(0, duration, int(duration * 5))

    pos = start_T[:3, -1]
    pos_des = end_T[:3, -1]
    pos_lst = np.array([pos, pos_des])

    cs = []
    pos_traj = []
    q_traj = []

    for i in range(len(pos)):
        cs.append(CubicSpline(t, pos_lst[:, i], bc_type='clamped'))

        pos_traj_individual = cs[i](ts)
        pos_traj.append(pos_traj_individual)
    pos_traj = np.array(pos_traj).T

    ori = R.from_matrix(start_T[:3, :3])
    ori_des = R.from_matrix(end_T[:3, :3])

    RS = RotationSpline(t, R.concatenate([ori, ori_des]))   # rotation vector + cubic spline
    # Limitation: The angular acceleration is continuous, but not smooth.

    ori_traj = RS(ts).as_matrix()
    # slerp = Slerp(t, R.concatenate([ori, ori_des]))
    # ori_traj = slerp(ts).as_matrix()
    Ts = np.zeros((len(pos_traj), 4, 4))
    Ts[:, -1, -1] = 1
    Ts[:, :3, -1] = pos_traj
    Ts[:, :3, :3] = ori_traj

    for T in Ts:
        q_traj.append(panda.ik(T))
    q_traj = np.array(q_traj)


    if visualize:
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.add_subplot(121, projection='3d')
        ax.scatter(Ts[0, 0, -1], Ts[0, 1, -1], Ts[0, 2, -1], s=30, label='start')
        ax.scatter(Ts[-1, 0, -1], Ts[-1, 1, -1], Ts[-1, 2, -1], s=30, label='end')
        for T in Ts:
            position = T[:3, -1]
            orientation = T[:3, :3]
            ax.scatter(position[0], position[1], position[2], s=5, c='black')

            axis_length = 0.05
            x_axis = orientation[:, 0] * axis_length
            y_axis = orientation[:, 1] * axis_length
            z_axis = orientation[:, 2] * axis_length

            ax.quiver(position[0], position[1], position[2], x_axis[0], x_axis[1], x_axis[2], color='r', length=axis_length, normalize=True)
            ax.quiver(position[0], position[1], position[2], y_axis[0], y_axis[1], y_axis[2], color='g', length=axis_length, normalize=True)
            ax.quiver(position[0], position[1], position[2], z_axis[0], z_axis[1], z_axis[2], color='b', length=axis_length, normalize=True)

        ax2 = fig.add_subplot(122)
        for i in range(len(q_traj[0])):
            ax2.scatter(ts, q_traj[:, i], s=2)

        # Set labels and display the plot
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_aspect('equal')

        ax.legend()
        plt.show()

    return ts, q_traj


if __name__ == "__main__":
    ## Joint Space
    q0 = np.array([0, 0, 0, 0, 0, 0, 0])
    q1 = np.array([2, 3, 4, 5, 6, 10, 5])
    q2 = np.array([2, -1, 1, 5, 0, 15, 10])
    q3 = np.array([10, 1, 2, 10, -2, 0, 1])
    duration = 4


    q_lst = [q0, q1, q2, q3]
    q_lst = np.array(q_lst).reshape(len(q_lst), -1)

    ts, q_traj = interpolate_q([q0, q1, q2, q3], duration, visualize=True)


    ## Cartesian Space
    Tb_ed = np.array([
        [ 1.     ,  0.     ,  0.     ,  0.55233],
        [ 0.     , -1.     , -0.     ,  0.06299],
        [ 0.     ,  0.     , -1.     ,  0.37146],
        [ 0.     ,  0.     ,  0.     ,  1.     ]])

    Tb_ee = np.array([
        [0.94829, 0.16114, 0.27347, 0.38523],
        [0.18755, -0.97953, -0.07316, 0.12156],
        [0.25608, 0.12067, -0.95909, 0.6306],
        [0., 0., 0., 1.]])

    ts, q_traj = interpolate_T(Tb_ee, Tb_ed, duration=7, visualize=True)