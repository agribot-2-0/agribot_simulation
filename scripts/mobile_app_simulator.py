#!/usr/bin/env python

import os
import signal
import sys
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QApplication  # pylint: disable=no-name-in-module

from industrial_msgs.msg import RobotMode
from std_msgs.msg import String
from std_srvs.srv import Empty
from agribot_msgs.msg import AgribotAppErrorFlag
from agribot_msgs.msg import AgribotManualControl
from agribot_msgs.msg import AgribotObstacleDetection
from agribot_msgs.msg import AgribotTool
from agribot_msgs.srv import ToolManagerState
from agribot_msgs.srv import ToolManagerStateRequest

import rospy
import rospkg


class MobileAppSim(QWidget):
    # pylint: disable=too-many-instance-attributes
    max_linear_velocity = 2.0
    max_circle_radius = 11.0
    no_tags = "0"
    send_tpl = False

    def __init__(self, parent=None):
        super(MobileAppSim, self).__init__(parent)
        rospy.init_node('mobile_app_simulator')

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('agribot_simulation')
        ui_file = os.path.join(pkg_path, 'scripts', 'ui', 'mobile_app_sim.ui')
        loadUi(ui_file, self)
        self.setWindowTitle("Mobile App Simulator")

        self.mode = rospy.Publisher('/agribot/state_machine/mode',
                                    RobotMode,
                                    queue_size=1)

        self.error = rospy.Publisher('/agribot/state_machine/error',
                                     AgribotAppErrorFlag,
                                     queue_size=1)
        self.task = rospy.Publisher('/agribot/state_machine/task',
                                    String,
                                    queue_size=1)

        self.obstacle = rospy.Publisher('/agribot/obstacle_detected',
                                        AgribotObstacleDetection,
                                        queue_size=1)

        self.manual_control = rospy.Publisher('cmd_vel_manual',
                                              AgribotManualControl,
                                              queue_size=10)

        self.front_tpl = rospy.Publisher('/agribot/control/tool/front',
                                         AgribotTool,
                                         queue_size=1)

        self.back_tpl = rospy.Publisher('/agribot/control/tool/back',
                                        AgribotTool,
                                        queue_size=1)

        try:
            self.tags_scan = rospy.ServiceProxy('/agribot/tool_management_state',
                                                ToolManagerState)
        except rospy.ServiceException as exc:
            rospy.logerr("Service init failed: {}".format(exc))

        # Buttons init
        self.button_send_task.clicked.connect(self.send_task)
        self.button_autonomous.clicked.connect(self.autonomous_mode)
        self.button_manual.clicked.connect(self.manual_mode)
        self.button_error_frame.clicked.connect(self.send_error_frame)
        self.button_error_manual.clicked.connect(self.send_manual_error)
        self.button_no_error.clicked.connect(self.send_no_error)
        self.button_obstacle.clicked.connect(self.obstacle_detected)
        self.button_no_obstacle.clicked.connect(self.no_obstacle_detected)
        self.button_reset_gazebo.clicked.connect(self.gazebo_reset_world)
        self.button_scan_tags.clicked.connect(self.scan_for_tags)
        self.button_send_tpl.clicked.connect(self.set_tpl)
        self.manual_msg = AgribotManualControl()

        # List init
        self.back_tags.addItem(self.no_tags)
        self.back_tags.addItem(self.no_tags)
        self.front_tags.addItem(self.no_tags)
        self.front_tags.addItem(self.no_tags)

        # Sliders init
        self.slider_tpl_angle_fr.valueChanged.connect(self.val_change_tpl_angle_fr)
        self.slider_tpl_height_fr.valueChanged.connect(self.val_change_tpl_height_fr)
        self.slider_red_valve_flow_fr.valueChanged.connect(self.val_change_red_valve_flow_fr)
        self.slider_green_valve_flow_fr.valueChanged.connect(self.val_change_green_valve_flow_fr)
        self.slider_blue_valve_flow_fr.valueChanged.connect(self.val_change_blue_valve_flow_fr)
        self.slider_pto_rpm_fr.valueChanged.connect(self.val_change_pto_rpm_fr)
        self.slider_tpl_angle_bk.valueChanged.connect(self.val_change_tpl_angle_bk)
        self.slider_tpl_height_bk.valueChanged.connect(self.val_change_tpl_height_bk)
        self.slider_red_valve_flow_bk.valueChanged.connect(self.val_change_red_valve_flow_bk)
        self.slider_green_valve_flow_bk.valueChanged.connect(self.val_change_green_valve_flow_bk)
        self.slider_blue_valve_flow_bk.valueChanged.connect(self.val_change_blue_valve_flow_bk)
        self.slider_pto_rpm_bk.valueChanged.connect(self.val_change_pto_rpm_bk)

        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        timestamp_thread = threading.Thread(target=self.rosloop)
        timestamp_thread.setDaemon(True)
        timestamp_thread.start()

    def send_task(self):
        msg = String()
        msg.data = self.text_task_name.text()
        self.task.publish(msg)

    def autonomous_mode(self):
        msg = RobotMode()
        msg.val = RobotMode.AUTO
        self.mode.publish(msg)

    def manual_mode(self):
        msg = RobotMode()
        msg.val = RobotMode.MANUAL
        self.mode.publish(msg)

    def send_error_frame(self):
        msg = AgribotAppErrorFlag()
        msg.error_flag = AgribotAppErrorFlag.APP_PROTOCOL_ERROR
        self.error.publish(msg)

    def send_manual_error(self):
        msg = AgribotAppErrorFlag()
        msg.error_flag = AgribotAppErrorFlag.APP_MANUAL_ERROR
        self.error.publish(msg)

    def send_no_error(self):
        msg = AgribotAppErrorFlag()
        msg.error_flag = AgribotAppErrorFlag.NO_APP_ERRORS
        self.error.publish(msg)

    def obstacle_detected(self):
        msg = AgribotObstacleDetection()
        msg.status = AgribotObstacleDetection.OBSTACLE_DETECTED
        self.obstacle.publish(msg)

    def no_obstacle_detected(self):
        msg = AgribotObstacleDetection()
        msg.status = AgribotObstacleDetection.NO_OBSTACLE
        self.obstacle.publish(msg)

    def gazebo_reset_world(self):
        self.reset_world()

    def scan_for_tags(self):
        self.front_tags.clear()
        self.back_tags.clear()
        try:
            resp = self.tags_scan(ToolManagerStateRequest.SCAN_FRONT)
        except rospy.ServiceException as exc:
            rospy.logerr("Service call failed: {}".format(exc))
        else:
            for tag in resp.ids:
                self.front_tags.addItem(str(tag))
        try:
            resp = self.tags_scan(ToolManagerStateRequest.SCAN_BACK)
        except rospy.ServiceException as exc:
            rospy.logerr("Service call failed: {}".format(exc))
        else:
            for tag in resp.ids:
                self.back_tags.addItem(str(tag))

    def set_tpl(self):
        try:
            self.tags_scan(ToolManagerStateRequest.SET_TPL)
        except rospy.ServiceException as exc:
            rospy.logerr("Service call failed: {}".format(exc))
            self.send_tpl = False
        else:
            self.send_tpl = True

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

    def send_tpl_config(self):
        msg = AgribotTool()
        msg.tpl_angle = self.slider_tpl_angle_fr.value()
        msg.tpl_height = self.slider_tpl_height_fr.value()
        msg.red_valve_flow = self.slider_red_valve_flow_fr.value()
        msg.green_valve_flow = self.slider_green_valve_flow_fr.value()
        msg.blue_valve_flow = self.slider_blue_valve_flow_fr.value()
        msg.pto_rpm = self.slider_pto_rpm_fr.value()
        self.front_tpl.publish(msg)
        msg.tpl_angle = self.slider_tpl_angle_bk.value()
        msg.tpl_height = self.slider_tpl_height_bk.value()
        msg.red_valve_flow = self.slider_red_valve_flow_bk.value()
        msg.green_valve_flow = self.slider_green_valve_flow_bk.value()
        msg.blue_valve_flow = self.slider_blue_valve_flow_bk.value()
        msg.pto_rpm = self.slider_pto_rpm_bk.value()
        self.back_tpl.publish(msg)

    def rosloop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.manual_msg.linear_velocity = (
                self.slider_linear.value() *
                self.max_linear_velocity) / self.slider_linear.maximum()
            self.manual_msg.circle_radius = (
                self.slider_angular.value() *
                self.max_circle_radius) / self.slider_angular.maximum()
            self.manual_control.publish(self.manual_msg)
            if self.send_tpl:
                self.send_tpl_config()
            rate.sleep()


def main():
    app = QApplication(sys.argv)
    ex = MobileAppSim()
    ex.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()
