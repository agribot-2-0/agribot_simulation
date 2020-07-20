#!/usr/bin/env python

import os
import signal
import sys

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QApplication

from industrial_msgs.msg import RobotMode
from std_msgs.msg import String
from agribot_msgs.msg import AgribotAppErrorFlag
from agribot_msgs.msg import AgribotManualControl
from agribot_msgs.msg import AgribotObstacleDetection

import rospy
import rospkg

class MobileAppSim(QWidget):
    def __init__(self, parent=None):
        super(MobileAppSim, self).__init__(parent)
        rospy.init_node('mobile_app_simulator')

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('agribot_simulation')
        ui_file = os.path.join(pkg_path,
                               'scripts',
                               'ui',
                               'mobile_app_sim.ui')
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

        self.button_send_task.clicked.connect(self.send_task)
        self.button_autonomous.clicked.connect(self.autonomous_mode)
        self.button_manual.clicked.connect(self.manual_mode)
        self.button_error_frame.clicked.connect(self.send_error_frame)
        self.button_error_manual.clicked.connect(self.send_manual_error)
        self.button_no_error.clicked.connect(self.send_no_error)
        self.button_obstacle.clicked.connect(self.obstacle_detected)
        self.button_no_obstacle.clicked.connect(self.no_obstacle_detected)
        self.slider_linear.valueChanged.connect(self.send_velocity_linear)
        self.slider_angular.valueChanged.connect(self.send_velocity_angular)

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

    def send_velocity_linear(self):
        msg = AgribotManualControl()
        msg.linear_velocity = self.slider_linear.value()/10.0
        msg.circle_radius = self.slider_angular.value()/10.0
        self.manual_control.publish(msg)

    def send_velocity_angular(self):
        msg = AgribotManualControl()
        msg.linear_velocity = self.slider_linear.value()/10.0
        msg.circle_radius = self.slider_angular.value()/10.0
        self.manual_control.publish(msg)

def main():
    app = QApplication(sys.argv)
    ex = MobileAppSim()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()
