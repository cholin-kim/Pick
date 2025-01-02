from dynamixel.Dynamixel import Dynamixel
from panda_teleassembly.utils.Trajectory import Trajectory
import copy
import numpy as np
import time


def motor_tic_to_ang(tic):
    return np.array(tic) * (2*np.pi) / 4096


def motor_ang_to_tic(ang):
    return np.array(ang) * 4096 / (2*np.pi)


class ThreeFingerGrasperDX:
    def __init__(self, port='/dev/ttyUSB0'):
        self.dx = Dynamixel(device_name=port)

        # Variables
        self.max_dq = 2.0
        self.gear_ratio = 56./36.  # shaft gear : DX gear

        # Set grasper limits
        self.jaw_limit_max = np.array(np.deg2rad(160))
        self.jaw_limit_min = np.array(np.deg2rad(0))

        # Initialize grasper
        self.init_dx()

    def __del__(self):
        # self.set_joint(func=np.deg2rad([0.0, 0.0, 0.0]))
        print ("Disabling motors")
        self.enable_motors(False)

    def init_dx(self):
        # Disable all motors
        print("Disabling motors")
        self.enable_motors(False)

        # Set position mode and switching to extended-position mode
        print("Set extended-position mode")
        self.set_operating_mode(mode=4)

        # Set neutral position
        self.neutral_m1 = 1702
        self.neutral_m2 = 4178
        neutral_center = (self.neutral_m1+self.neutral_m2)/2
        current_tic = np.array(self.get_curr_motor_tic(option='absolute'))
        print("Current dx pos=", current_tic)
        print("Neutral center=", neutral_center)

        crit = (int(neutral_center * 2) - current_tic[0]) - current_tic[1]
        if crit > 4096/2:
            print("-")
            neutral_center -= 4096/2
        elif crit < -4096/2:
            print("+")
            neutral_center += 4096/2

        self.dx_pos_neutral = current_tic
        self.dx_pos_neutral[1] = int(neutral_center * 2) - self.dx_pos_neutral[0]
        print("Current dx pos=", self.get_curr_motor_tic(option='absolute'))
        print("Neutral dx pos=", self.dx_pos_neutral)

        # Enable all motors
        print("Enabling motors")
        self.enable_motors(True)

        # Set grasper neutral
        self.set_joint(joints=[0, 0])  # (rad)

    """
    DXL functions
    """
    def set_operating_mode(self, mode):
        for i in range(1, 3):
            self.dx.set_operating_mode(dxl_id=i, mode=mode)

    def enable_motors(self, flag):
        for i in range(1, 3):
            self.dx.enable_led(dxl_id=i, flag=flag)
            self.dx.enable_torque(dxl_id=i, flag=flag)

    """
    Get States
    """
    def get_curr_motor_tic(self, option='nominal'):
        tic_temp = self.dx.get_pos_sync(dxl_ids=[*range(1, 3)])
        tic = [t if t < 0xF0000000 else t-0xFFFFFFFF for t in tic_temp]
        if option == 'nominal':
            tic = np.array(tic) - np.array(self.dx_pos_neutral)
        tic = np.array(tic).astype(int)
        return tic

    def get_curr_motor(self):
        """
        :return: current motor angle (rad)
        """
        return motor_tic_to_ang(self.get_curr_motor_tic())

    def get_curr_joint(self):
        """
        :return: current joint angle (rad)
        """
        motors = self.get_curr_motor()
        return self.motor_to_joint(motors)

    """
    Set Targets
    """
    def set_motor_target_direct(self, ang):
        """
        motor target in angles (rad)
        """
        tic = self.dx_pos_neutral + motor_ang_to_tic(ang)
        tic = tic.astype(int)
        self.dx.set_pos_sync(dxl_ids=[*range(1, 3)], goal_positions=tic)

    def set_joint_direct(self, joints):
        """
        set functions directly (with full speed)
        func:  pitch, yaw, jaw
        """
        assert not np.isnan(np.sum(joints))
        motor = self.joint_to_motor(joints=joints)
        self.set_motor_target_direct(motor)

    def set_joint(self, joints, interpolation='cubic', tf=None, t_step=0.01):
        """
        set functions following cubic trajectory
        func:  pitch, yaw, jaw
        alpha:  coefficient related to speed of the trajectory. (the larger, the slower)
        t_step: time step to interpolate between via points
        """
        assert not np.isnan(np.sum(joints))

        # Define q0 & qf
        q0 = self.get_curr_joint()
        qf = joints

        if np.allclose(qf, q0, rtol=1e-01, atol=1e-01):
            return False

        # Define tf proportional to the norm(funcf-func0)
        if tf is None:
            max_movement = np.max(np.abs(np.array(qf) - np.array(q0)))
            tf = max_movement / self.max_dq
        if interpolation == 'cubic':
            pos, t = Trajectory.cubic(q0=q0, qf=qf, v0=np.zeros(2), vf=np.zeros(2), tf=tf, t_step=t_step)
        else:
            pos, t = Trajectory.quintic(q0=q0, qf=qf, v0=np.zeros(2), vf=np.zeros(2), a0=np.zeros(2), af=np.zeros(2), tf=tf, t_step=t_step)

        # Execute trajectory
        for q in pos:
            self.set_joint_direct(joints=q)
            # print(self.dx.get_current_sync(dxl_ids=[1, 2]))
            time.sleep(t_step)
        return True


    """
    Conversion Functions
    """

    def joint_to_motor(self, joints):  # convert joint angle into motor angle
        """
        joints: q1, q2 (rad)
        return: m1, m2 (rad)
        """
        q1, q2 = joints     # (roll, jaw)

        # thresholding by motion range
        if q2 < self.jaw_limit_min:
            q2 = self.jaw_limit_min
        elif q2 > self.jaw_limit_max:
            q2 = self.jaw_limit_max

        m1 = -q1/self.gear_ratio
        m2 = (q1-q2)/self.gear_ratio
        return [m1, m2]

    def motor_to_joint(self, motors):
        """
        motors: m1, m2 (rad)
        return: q1, q2 (rad) (roll, jaw)
        """
        m1, m2 = motors
        q1 = -m1*self.gear_ratio        # roll angle
        q2 = -(m2+m1)*self.gear_ratio    # jaw angle
        return [q1, q2]


if __name__ == "__main__":
    from dynamixel.PortFinder import PortFinder
    pf = PortFinder()
    port = pf.find_port(dxl_ids=[1, 2], secondary_ids=[1, 2])
    grasper = ThreeFingerGrasperDX(port=port)
    while True:
        grasper.set_joint(joints=np.deg2rad([0.0, 0.0]))