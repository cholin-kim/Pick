import rospy
from controller_manager_msgs.srv import (SwitchController, SwitchControllerRequest, LoadController, LoadControllerRequest,
                                         UnloadController, UnloadControllerRequest, ListControllers, ListControllerTypes, ListControllersRequest)
# from controller_manager_msgs.msg import ControllerState


class Controller_Switch():
    def __init__(self, des_ctrl = "cartesian_impedence_example_controller"):
        # rospy.init_node('controller_switcher')

        self.fs_str = "franka_state_controller"
        self.cic_str = "cartesian_impedance_example_controller"
        self.pjtc_str = "position_joint_trajectory_controller"
        self.ejtc_str = "effort_joint_trajectory_controller"

        rospy.wait_for_service('/controller_manager/switch_controller')
        rospy.wait_for_service('/controller_manager/load_controller')
        rospy.wait_for_service('/controller_manager/list_controllers')

        load_controller = rospy.ServiceProxy('/controller_manager/load_controller', LoadController)
        self.list_controller = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        self._switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        load_controller(self.pjtc_str)
        load_controller(self.ejtc_str)
        load_controller(self.fs_str)

        self.current_controller = None
        self.find_current_controller()

        self.req = SwitchControllerRequest()
        self.req.strictness = SwitchControllerRequest.STRICT
        self.req.start_asap = True
        self.req.timeout = 0.0
        # self.start_controller(self.fs_str)

    def find_current_controller(self):
        response = self.list_controller()
        active_controllers = [controller.name for controller in response.controller if controller.state == 'running']
        self.current_controller = active_controllers

    def start_controller(self, des_controller):
        self.req.start_controllers = [des_controller]
        self.req.stop_controllers = []
        try:
            result = self._switch_controller(self.req)
            if result.ok:
                print('done')
            else:
                print('failed')
        except rospy.ServiceException as e:
            print(e)

    def switch_controller(self, des_controller):
        if des_controller == self.current_controller[-1]:
            print(f'The controller named {des_controller} is already activated.')
        else:
            self.req.start_controllers = [des_controller]
            self.req.stop_controllers = [item for item in self.current_controller if not item.startswith('f')]
            try:
                result = self._switch_controller(self.req)
                if result.ok:
                    print('done')
                else:
                    print('failed')
            except rospy.ServiceException as e:
                print(e)



if __name__ == "__main__":
    sc = Controller_Switch()
    sc.switch_controller('effort_joint_trajectory_controller')
