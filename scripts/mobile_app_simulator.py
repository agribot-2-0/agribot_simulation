#!/usr/bin/env python

import os
import signal
import sys

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QApplication

from industrial_msgs.msg import RobotMode

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

        self.button_autonomous.clicked.connect(self.autonomous_mode)
        self.button_manual.clicked.connect(self.manual_mode)
        self.button_error_frame.clicked.connect(self.send_error_frame)
        self.button_error_manual.clicked.connect(self.send_manual_error)
        self.button_obstacle.clicked.connect(self.obstacle_detected)

    def autonomous_mode(self):
        msg = RobotMode()
        msg.val = RobotMode.AUTO
        self.mode.publish(msg)

    def manual_mode(self):
        msg = RobotMode()
        msg.val = RobotMode.MANUAL
        self.mode.publish(msg)

    def send_error_frame(self):
        pass

    def send_manual_error(self):
        pass

    def obstacle_detected(self):
        pass

def main():
    app = QApplication(sys.argv)
    ex = MobileAppSim()
    ex.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    main()
