import sys
import signal
import rclpy

from rclpy.node import Node

from PyQt5.QtWidgets import QCheckBox

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QLabel,
    QRadioButton,
    QButtonGroup,
)
from std_msgs.msg import Bool
from std_msgs.msg import String

from std_srvs.srv import Empty

from PyQt5.QtCore import QTimer


class ControlPanel(QWidget):

    def __init__(self, node):

        super().__init__()

        self.node = node

        self.setWindowTitle(
            "Bulldozer Control Panel"
        )

        layout = QVBoxLayout()

        # AUTO

        self.auto_toggle = QCheckBox(
            "AUTO"
        )

        self.auto_toggle.stateChanged.connect(
            self.auto_changed
        )

        layout.addWidget(
            self.auto_toggle
        )

        # LOOP

        self.loop_toggle = QCheckBox(
            "MISSION LOOP"
        )

        self.loop_toggle.stateChanged.connect(
            self.loop_changed
        )

        layout.addWidget(
            self.loop_toggle
        )

        # ACTION

        layout.addWidget(
            QLabel("ACTION")
        )

        self.action_label = QLabel(
            "Current: MOVE"
        )

        self.status_label = QLabel(
            "Waiting..."
        )

        layout.addWidget(
            self.status_label
        )

        layout.addWidget(
            self.action_label
        )

        self.action_group = QButtonGroup()

        self.move_radio = (
            QRadioButton("MOVE")
        )

        self.scoop_radio = (
            QRadioButton("SCOOP")
        )

        self.dump_radio = (
            QRadioButton("DUMP")
        )

        self.move_radio.setChecked(
            True
        )

        self.action_group.addButton(
            self.move_radio
        )

        self.action_group.addButton(
            self.scoop_radio
        )

        self.action_group.addButton(
            self.dump_radio
        )

        layout.addWidget(
            self.move_radio
        )

        layout.addWidget(
            self.scoop_radio
        )

        layout.addWidget(
            self.dump_radio
        )

        self.move_radio.toggled.connect(
            lambda checked:
            checked and self.set_action("MOVE")
        )

        self.scoop_radio.toggled.connect(
            lambda checked:
            checked and self.set_action("SCOOP")
        )

        self.dump_radio.toggled.connect(
            lambda checked:
            checked and self.set_action("DUMP")
        )

        # SERVICES

        save_btn = QPushButton(
            "SAVE"
        )

        load_btn = QPushButton(
            "LOAD"
        )

        undo_btn = QPushButton(
            "UNDO"
        )

        undo_btn.clicked.connect(
            self.undo_waypoint
        )

        layout.addWidget(
            undo_btn
        )

        clear_btn = QPushButton(
            "CLEAR"
        )

        save_btn.clicked.connect(
            self.save_mission
        )

        load_btn.clicked.connect(
            self.load_mission
        )

        clear_btn.clicked.connect(
            self.clear_mission
        )

        layout.addWidget(save_btn)
        layout.addWidget(load_btn)
        layout.addWidget(clear_btn)

        self.setLayout(layout)

        self.timer = QTimer()

        self.timer.timeout.connect(
            self.update_status
        )

        self.timer.start(100)

    def set_action(
        self,
        action
    ):

        msg = String()

        msg.data = action

        self.node.action_pub.publish(
            msg
        )
        self.action_label.setText(
            f"Current: {action}"
        )

    def save_mission(self):

        self.node.save_client.call_async(
            Empty.Request()
        )

    def load_mission(self):

        self.node.load_client.call_async(
            Empty.Request()
        )

    def clear_mission(self):

        self.node.clear_client.call_async(
            Empty.Request()
        )
    
    def auto_changed(self, state):

        msg = Bool()

        msg.data = bool(state)

        self.node.auto_pub.publish(
            msg
        )

    def loop_changed(self, state):

        msg = Bool()

        msg.data = bool(state)

        self.node.loop_pub.publish(
            msg
        )

    def update_status(self):

        self.status_label.setText(
            self.node.status_text
        )

    def undo_waypoint(self):

        self.node.undo_client.call_async(
            Empty.Request()
        )

    



class GuiNode(Node):

    def __init__(self):

        super().__init__(
            "control_panel"
        )

        self.auto_pub = (
            self.create_publisher(
                Bool,
                "/auto_enable",
                10
            )
        )

        self.loop_pub = (
            self.create_publisher(
                Bool,
                "/mission_loop",
                10
            )
        )

        self.action_pub = (
            self.create_publisher(
                String,
                "/next_action",
                10
            )
        )

        self.save_client = (
            self.create_client(
                Empty,
                "/save_mission"
            )
        )

        self.load_client = (
            self.create_client(
                Empty,
                "/load_mission"
            )
        )

        self.clear_client = (
            self.create_client(
                Empty,
                "/clear_mission"
            )
        )

        

        self.status_text = "Waiting..."

        self.status_sub = (
            self.create_subscription(
                String,
                "/robot_status",
                self.status_callback,
                10
            )
        )

        self.undo_client = (
            self.create_client(
                Empty,
                "/undo_waypoint"
            )
        )
    
    def status_callback(self, msg):

        self.status_text = msg.data
    


def main():

    rclpy.init()

    node = GuiNode()

    app = QApplication(
        sys.argv
    )

    panel = ControlPanel(
        node
    )

    panel.show()

    msg = String()
    msg.data = "MOVE"
    node.action_pub.publish(msg)

    ros_timer = QTimer()

    ros_timer.timeout.connect(
        lambda:
        rclpy.spin_once(
            node,
            timeout_sec=0
        )
    )

    ros_timer.start(50)

    signal.signal(
        signal.SIGINT,
        signal.SIG_DFL
    )

    app.exec()

    rclpy.shutdown()


if __name__ == "__main__":

    main()