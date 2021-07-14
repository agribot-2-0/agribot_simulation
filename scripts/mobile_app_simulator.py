#!/usr/bin/env python

import os
import signal
import sys
import threading
import math

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QApplication  # pylint: disable=no-name-in-module

import actionlib
from std_srvs.srv import Empty
from agribot_msgs.msg import AgribotTtcontrolState
from agribot_msgs.msg import AgribotAppErrorFlag
from agribot_msgs.msg import AgribotManualControl
from agribot_msgs.msg import ObstacleDetectionStatus
from agribot_msgs.msg import AgribotTool
from agribot_msgs.msg import PerformAutonomousTaskAction
from agribot_msgs.msg import PerformAutonomousTaskGoal
from agribot_msgs.srv import ToolManagerScan
from agribot_msgs.srv import ToolManagerScanRequest
from agribot_msgs.srv import SteeringMode
from agribot_msgs.srv import SteeringModeRequest

from agribot_common.agribot_parameters import get_parameter

import rospy
import rospkg

LOG_TAG = "MobileAppSimulator"


class MobileAppSim(QWidget):
    # pylint: disable=too-many-instance-attributes, too-many-statements, too-many-public-methods
    max_linear_velocity = 2.0
    max_circle_radius = 11.0
    no_tags = "0"
    min_tpl_angle = -1  # deg
    max_tpl_angle = 51  # deg
    max_tpl_height = 25  # cm

    def __init__(self, parent=None):
        super(MobileAppSim, self).__init__(parent)
        rospy.init_node('mobile_app_simulator')

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('agribot_simulation')
        ui_file = os.path.join(pkg_path, 'scripts', 'ui', 'mobile_app_sim.ui')
        loadUi(ui_file, self)
        self.setWindowTitle("Mobile App Simulator")

        self.ttcontrol_state_publisher = rospy.Publisher('/agribot/ttcontrol_state',
                                                         AgribotTtcontrolState,
                                                         queue_size=1)

        self.error = rospy.Publisher('/agribot/state_machine/error',
                                     AgribotAppErrorFlag,
                                     queue_size=1)

        self.task_action_client = actionlib.SimpleActionClient('/agribot/behavior_tree/task_action',
                                                               PerformAutonomousTaskAction)

        self.obstacle = rospy.Publisher('/agribot/obstacle_detection/front/status',
                                        ObstacleDetectionStatus,
                                        queue_size=1)

        self.manual_control = rospy.Publisher('cmd_vel_manual', AgribotManualControl, queue_size=1)

        self.front_tpl = rospy.Publisher('/agribot/control/tool/front/manual',
                                         AgribotTool,
                                         queue_size=1)

        self.back_tpl = rospy.Publisher("/agribot/control/tool/back/manual",
                                        AgribotTool,
                                        queue_size=1)

        try:
            self.tags_scan = rospy.ServiceProxy("/agribot/tool_management/scan", ToolManagerScan)
            self.kinematics_mode = rospy.ServiceProxy(
                "/agribot/kinematics_controller/steering_mode", SteeringMode)
            self.gazebo_reset = rospy.ServiceProxy("/gazebo/reset_world", Empty)
            self.gazebo_pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            self.gazebo_unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        except rospy.ServiceException as exc:
            rospy.logerr("[{}] Service init failed: {}".format(LOG_TAG, exc))

        # Buttons init
        self.button_send_task.clicked.connect(
            lambda: self.send_task(self.combo_task_name.currentText()))
        self.button_cancel_task.clicked.connect(self.cancel_task)
        self.button_manual.clicked.connect(self.send_ttcontrol_manual_state)
        self.button_safe.clicked.connect(self.send_ttcontrol_safe_state)
        self.button_error_frame.clicked.connect(
            lambda: self.send_error_frame(AgribotAppErrorFlag.APP_PROTOCOL_ERROR))
        self.button_error_manual.clicked.connect(
            lambda: self.send_error_frame(AgribotAppErrorFlag.APP_MANUAL_ERROR))
        self.button_no_error.clicked.connect(
            lambda: self.send_error_frame(AgribotAppErrorFlag.NO_APP_ERRORS))
        self.button_obstacle.clicked.connect(
            lambda: self.send_is_obstacle_detected(ObstacleDetectionStatus.OBSTACLE_DETECTED))
        self.button_no_obstacle.clicked.connect(
            lambda: self.send_is_obstacle_detected(ObstacleDetectionStatus.NO_OBSTACLE))
        self.button_reset_gazebo.clicked.connect(self.gazebo_reset_world)
        self.button_pause_gazebo.clicked.connect(self.gazebo_pause_world)
        self.button_unpause_gazebo.clicked.connect(self.gazebo_unpause_world)
        self.button_scan_tags.clicked.connect(self.scan_for_tags)
        self.button_4m_left.clicked.connect(lambda: self.set_circle_radius(4))
        self.button_2m_left.clicked.connect(lambda: self.set_circle_radius(2))
        self.button_0m.clicked.connect(lambda: self.set_circle_radius(0))
        self.button_2m_right.clicked.connect(lambda: self.set_circle_radius(-2))
        self.button_4m_right.clicked.connect(lambda: self.set_circle_radius(-4))
        self.button_1ms_forward.clicked.connect(lambda: self.set_linear(1))
        self.button_03ms_forward.clicked.connect(lambda: self.set_linear(0.3))
        self.button_0ms.clicked.connect(lambda: self.set_linear(0))
        self.button_03ms_backward.clicked.connect(lambda: self.set_linear(-0.3))
        self.button_1ms_backward.clicked.connect(lambda: self.set_linear(-1))
        self.button_kinematics_four.clicked.connect(
            lambda: self.set_kinematics(SteeringModeRequest.FOUR_WHEEL))
        self.button_kinematics_front.clicked.connect(
            lambda: self.set_kinematics(SteeringModeRequest.FRONT_WHEEL))
        self.button_kinematics_crab.clicked.connect(
            lambda: self.set_kinematics(SteeringModeRequest.CRAB))

        self.manual_msg = AgribotManualControl()

        # List init
        self.back_tags.addItem(self.no_tags)
        self.back_tags.addItem(self.no_tags)
        self.front_tags.addItem(self.no_tags)
        self.front_tags.addItem(self.no_tags)

        # Sliders init
        self.slider_tpl_angle_fr.valueChanged.connect(self.val_change_tpl_angle_fr)
        self.slider_tpl_angle_fr.setMinimum(self.min_tpl_angle)
        self.slider_tpl_angle_fr.setMaximum(self.max_tpl_angle)
        self.slider_tpl_height_fr.valueChanged.connect(self.val_change_tpl_height_fr)
        self.slider_tpl_height_fr.setMaximum(self.max_tpl_height)
        self.slider_red_valve_flow_fr.valueChanged.connect(self.val_change_red_valve_flow_fr)
        self.slider_green_valve_flow_fr.valueChanged.connect(self.val_change_green_valve_flow_fr)
        self.slider_blue_valve_flow_fr.valueChanged.connect(self.val_change_blue_valve_flow_fr)
        self.slider_pto_rpm_fr.valueChanged.connect(self.val_change_pto_rpm_fr)
        self.slider_tpl_angle_bk.valueChanged.connect(self.val_change_tpl_angle_bk)
        self.slider_tpl_angle_bk.setMinimum(self.min_tpl_angle)
        self.slider_tpl_angle_bk.setMaximum(self.max_tpl_angle)
        self.slider_tpl_height_bk.valueChanged.connect(self.val_change_tpl_height_bk)
        self.slider_tpl_height_bk.setMaximum(self.max_tpl_height)
        self.slider_red_valve_flow_bk.valueChanged.connect(self.val_change_red_valve_flow_bk)
        self.slider_green_valve_flow_bk.valueChanged.connect(self.val_change_green_valve_flow_bk)
        self.slider_blue_valve_flow_bk.valueChanged.connect(self.val_change_blue_valve_flow_bk)
        self.slider_pto_rpm_bk.valueChanged.connect(self.val_change_pto_rpm_bk)

        tasks_path = os.path.join(rospack.get_path("agribot_map"), "tasks")
        try:
            tasks_path = os.path.join(os.environ.get("AGRIBOT_HOME"),
                                      get_parameter("/agribot_map/tasks_path"))
        except ValueError as exception_msg:
            rospy.logerr("[{}] {}".format(LOG_TAG, exception_msg))
            return

        tasks = [
            os.path.splitext(f)[0] for f in os.listdir(tasks_path)
            if os.path.isfile(os.path.join(tasks_path, f))
        ]
        for item in tasks:
            self.combo_task_name.addItem(item)

        timestamp_thread = threading.Thread(target=self.rosloop)
        timestamp_thread.setDaemon(True)
        timestamp_thread.start()

    def send_task(self, task):
        goal = PerformAutonomousTaskGoal(task_name=task)
        self.task_action_client.send_goal(goal)

    def cancel_task(self):
        goal = PerformAutonomousTaskGoal(task_name="")
        self.task_action_client.send_goal(goal)

    def send_ttcontrol_manual_state(self):
        msg = AgribotTtcontrolState()
        msg.manual_state = AgribotTtcontrolState.STEERING_ACTIVE
        msg.automatic_state = AgribotTtcontrolState.COMMUNICATION_ACTIVE
        self.ttcontrol_state_publisher.publish(msg)

    def send_ttcontrol_safe_state(self):
        msg = AgribotTtcontrolState()
        msg.manual_state = AgribotTtcontrolState.COMMUNICATION_ACTIVE
        msg.automatic_state = AgribotTtcontrolState.COMMUNICATION_ACTIVE
        self.ttcontrol_state_publisher.publish(msg)

    def send_error_frame(self, error_type):
        msg = AgribotAppErrorFlag()
        msg.error_flag = error_type
        self.error.publish(msg)

    def send_is_obstacle_detected(self, is_detected):
        msg = ObstacleDetectionStatus()
        msg.status = is_detected
        self.obstacle.publish(msg)

    def gazebo_reset_world(self):
        try:
            self.gazebo_reset()
        except rospy.ServiceException as exception_msg:
            rospy.logerr("[{}]: {}".format(LOG_TAG, exception_msg))

    def gazebo_pause_world(self):
        try:
            self.gazebo_pause()
        except rospy.ServiceException as exception_msg:
            rospy.logerr("[{}]: {}".format(LOG_TAG, exception_msg))

    def gazebo_unpause_world(self):
        try:
            self.gazebo_unpause()
        except rospy.ServiceException as exception_msg:
            rospy.logerr("[{}]: {}".format(LOG_TAG, exception_msg))

    def scan_for_tags(self):
        self.front_tags.clear()
        self.back_tags.clear()
        try:
            resp = self.tags_scan(ToolManagerScanRequest.SCAN_FRONT)
        except rospy.ServiceException as exc:
            rospy.logerr("[{}] Service call failed: {}".format(LOG_TAG, exc))
        else:
            for tag in resp.ids:
                self.front_tags.addItem(str(tag))
        try:
            resp = self.tags_scan(ToolManagerScanRequest.SCAN_BACK)
        except rospy.ServiceException as exc:
            rospy.logerr("[{}] Service call failed: {}".format(LOG_TAG, exc))
        else:
            for tag in resp.ids:
                self.back_tags.addItem(str(tag))

    def val_change_tpl_angle_fr(self):
        self.tpl_angle_val_fr.setText(str(self.slider_tpl_angle_fr.value()))

    def val_change_tpl_height_fr(self):
        self.tpl_height_val_fr.setText(str(self.slider_tpl_height_fr.value()))

    def val_change_red_valve_flow_fr(self):
        self.red_valve_flow_val_fr.setText(str(self.slider_red_valve_flow_fr.value()))

    def val_change_blue_valve_flow_fr(self):
        self.blue_valve_flow_val_fr.setText(str(self.slider_blue_valve_flow_fr.value()))

    def val_change_green_valve_flow_fr(self):
        self.green_valve_flow_val_fr.setText(str(self.slider_green_valve_flow_fr.value()))

    def val_change_pto_rpm_fr(self):
        self.pto_rpm_val_fr.setText(str(self.slider_pto_rpm_fr.value()))

    def val_change_tpl_angle_bk(self):
        self.tpl_angle_val_bk.setText(str(self.slider_tpl_angle_bk.value()))

    def val_change_tpl_height_bk(self):
        self.tpl_height_val_bk.setText(str(self.slider_tpl_height_bk.value()))

    def val_change_red_valve_flow_bk(self):
        self.red_valve_flow_val_bk.setText(str(self.slider_red_valve_flow_bk.value()))

    def val_change_blue_valve_flow_bk(self):
        self.blue_valve_flow_val_bk.setText(str(self.slider_blue_valve_flow_bk.value()))

    def val_change_green_valve_flow_bk(self):
        self.green_valve_flow_val_bk.setText(str(self.slider_green_valve_flow_bk.value()))

    def val_change_pto_rpm_bk(self):
        self.pto_rpm_val_bk.setText(str(self.slider_pto_rpm_bk.value()))

    def cm_to_m(self, val):  # pylint: disable=no-self-use
        return val * 0.01

    def send_tpl_config(self):
        msg = AgribotTool()
        msg.tpl_angle = math.radians(self.slider_tpl_angle_fr.value())
        msg.tpl_height = self.cm_to_m(self.slider_tpl_height_fr.value())
        msg.red_valve_flow = self.slider_red_valve_flow_fr.value()
        msg.green_valve_flow = self.slider_green_valve_flow_fr.value()
        msg.blue_valve_flow = self.slider_blue_valve_flow_fr.value()
        msg.pto_rpm = self.slider_pto_rpm_fr.value()
        self.front_tpl.publish(msg)
        msg.tpl_angle = math.radians(self.slider_tpl_angle_bk.value())
        msg.tpl_height = self.cm_to_m(self.slider_tpl_height_bk.value())
        msg.red_valve_flow = self.slider_red_valve_flow_bk.value()
        msg.green_valve_flow = self.slider_green_valve_flow_bk.value()
        msg.blue_valve_flow = self.slider_blue_valve_flow_bk.value()
        msg.pto_rpm = self.slider_pto_rpm_bk.value()
        self.back_tpl.publish(msg)

    def get_linear(self):
        return (self.slider_linear.value() *
                self.max_linear_velocity) / self.slider_linear.maximum()

    def set_linear(self, linear_velocity):
        self.slider_linear.setValue(
            int((linear_velocity * self.slider_linear.maximum()) / self.max_linear_velocity))

    def get_circle_radius(self):
        return (self.slider_angular.value() *
                self.max_circle_radius) / self.slider_angular.maximum()

    def set_circle_radius(self, radius):
        self.slider_angular.setValue(
            int((radius * self.slider_angular.maximum()) / self.max_circle_radius))

    def set_kinematics(self, mode):
        try:
            self.kinematics_mode(mode)
        except rospy.ServiceException as exception_msg:
            rospy.logerr("[{}]: {}".format(LOG_TAG, exception_msg))

    def rosloop(self):
        try:
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                self.manual_msg.linear_velocity = self.get_linear()
                self.manual_msg.circle_radius = self.get_circle_radius()
                self.manual_control.publish(self.manual_msg)
                self.send_tpl_config()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass


def main():
    app = QApplication(sys.argv)
    ex = MobileAppSim()
    ex.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()
