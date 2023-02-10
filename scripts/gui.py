#!/usr/bin/env python3

import sys
import rospy
import psutil
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu, PointCloud2
from flux_msgs.msg import Frame
import subprocess
import cv2
import numpy as np


class DiagnosticWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        # Default values
        self.power_state = None
        self.connected = []
        self.sensors = {'ouster lidar':'167.2.11.2'}

        # For checking sensors
        self.sensor_topics = ['test', '/ML/Ouster/points', '/ML/Imu/Data', '/DL/RightImgFork',
                              '/DL/LeftImgFork', "/DL/LeftImgFront", '/DL/RightImgFront']
        self.msg_types = [Float32MultiArray,
                          PointCloud2, Imu, Frame, Frame, Frame, Frame]

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/pose", Pose, self.pose_callback)
        rospy.Subscriber("/Inputs", Float32MultiArray, self.power_callback)

        # Set up a timer to regularly update the status
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_robot_state)
        self.timer.timeout.connect(self.update_system_status)
        # self.timer.timeout.connect(self.update_sensor_data)
        # self.timer.timeout.connect(self.pose_callback)
        self.timer.timeout.connect(self.update_power_label)
        self.timer.timeout.connect(self.update_pallet_detection)
        self.timer.timeout.connect(self.update_obstacle_label)
        # self.timer.timeout.connect(self.update_map_viz)
        self.timer.start(1000)
        self.update_sensor_health_label()
        # self.sensor_data_timer = QTimer(self)
        # self.sensor_data_timer.timeout.connect(self.update_sensor_data)
        # self.sensor_data_timer.start(100)

    def initUI(self):
        # Create labels to display information
        self.robot_state_label = QLabel("Robot State: Unknown")
        self.power_label = QLabel("Power Consumption: None")
        self.system_status_label = QLabel("System Status: Unknown")
        self.sensor_health_label = QLabel("Sensor Connected: None")
        self.sensor_data_label = QLabel("Sensor Data: None")
        self.map_label = QLabel("Map Visualization: None")
        self.pallet_detection = QLabel("Pallet Detection: None")
        self.obstacle_label = QLabel("Obstacle : None")

        # Create a vertical layout to arrange the labels
        vbox = QVBoxLayout()
        vbox.addWidget(self.robot_state_label)
        vbox.addWidget(self.power_label)
        vbox.addWidget(self.system_status_label)
        vbox.addWidget(self.sensor_health_label)
        vbox.addWidget(self.sensor_data_label)
        vbox.addWidget(self.pallet_detection)
        vbox.addWidget(self.obstacle_label)
        # vbox.addWidget(self.map_label)

        # Set the layout for the widget
        self.setLayout(vbox)

        # Set the window title
        self.setWindowTitle("Diagnostic Window")

    def update_robot_state(self):
        # Retrieve the robot_state parameter
        state = rospy.get_param("state", "")
        self.robot_state_label.setText(
            "<b>Robot State</b>: <span style='color: green'> {} </span>".format(state))

    def update_sensor_health_label(self):
        for sensor in self.sensors:
            try:
                subprocess.check_output(
                    ["ping", "-c", "1", self.sensors[sensor]])
                # print(True)
                self.connected.append(sensor)
            except subprocess.CalledProcessError:
                # print(False)
                pass
        self.sensor_health_label.setText(
            "<b>Sensors connected </b>:<span style='color: green'>" + ", </span>".join(self.connected))

    def update_map_viz(self):
        # img = cv2.imread("cat.jpg")
        # cv2.imshow("image", img)
        pixmap = QPixmap("cat.jpg")
        self.map_label.setPixmap(pixmap)

    def update_power_label(self):
        # Retrieve the power state
        self.power_label.setText(
            "<b>Power cosumption</b>: <span style='color: red'>{}</span> ".format(self.power_state))


    def update_system_status(self):
        cpu_percent = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        disk_usage = psutil.disk_usage("/")

        system_status = "<b>CPU Usage</b>: <span style='color: yellow'>{:.2f}%</span> \n".format(
            cpu_percent)
        system_status += "<b>Memory Usage</b>: <span style='color: yellow'>{:.2f} GB / {:.2f} GB</span> \n".format(
            memory_info.used / 1024**3, memory_info.available / 1024**3)
        system_status += "<b>Disk Usage</b>: <span style='color: yellow'>{:.2f} GB / {:.2f} GB</span> \n".format(
            disk_usage.used / 1024**3, disk_usage.total / 1024**3)

        self.system_status_label.setText(system_status)

    def update_pallet_detection(self):
        self.pallet_detect = rospy.get_param("/CT/pallet_present")
        self.pallet_detection.setText(
            "<b>Pallet detected </b>: {}".format(self.pallet_detect))

    def update_obstacle_label(self):
        self.device_name = ['lidar center',
                            " lidar left", " lidar right", "camera"]
        self.device = [False, False, False, False]
        self.device[0] = rospy.get_param("stop/lidar_center")
        self.device[1] = rospy.get_param("stop/lidar_left")
        self.device[2] = rospy.get_param("stop/lidar_right")
        self.device[3] = rospy.get_param("stop/camera")

        if not any(self.device):
            self.obstacle_label.setText("<b>No Obstacle detected<b>")
        else:
            idx = [i for i, x in enumerate(self.device) if x]
            obstacles_loc = [self.device_name[i] for i in idx]
            self.obstacle_label.setText(
                "<b>Obstacle at: </b>: - " + ", ".join(obstacles_loc))

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width))
        self.map_image = (self.map_data * 255).astype(np.uint8)
        self.map_image = cv2.resize(self.map_image, (300, 300),
                                    interpolation=cv2.INTER_NEAREST)
        self.map_image = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2RGB)

        # map_image = QImage(
        # map_image, map_image.shape[1], map_image.shape[0], QImage.Format_RGB888)
        # self.map_visualization_label.setPixmap(QPixmap.fromImage(map_image))

    def pose_callback(self, msg):
        # Extract the x, y, and orientation information from the Pose message
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation = msg.pose.orientation

        # Convert the localization information into a form that can be displayed on the GUI
        # For example, you can use a red dot to represent the robot's position
        dot_radius = 5
        dot_color = (255, 0, 0)
        cv2.circle(self.map_image, (x, y), dot_radius, dot_color, -1)

        # Update the map display
        map_image = cv2.cvtColor(self.map_data, cv2.COLOR_GRAY2RGB)
        height, width, channel = map_image.shape
        bytesPerLine = 3 * width
        # qImg = QImage(map_image.data, width, height,
        #               bytesPerLine, QImage.Format_RGB888)
        # pixmap = QPixmap.fromImage(qImg)

        pixmap = QPixmap("cat.jpg")
        self.map_label.setPixmap(pixmap)

    def power_callback(self, msg):
        self.power_state = msg.data[22]


if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("diagnostic_window")

    # Start the Qt application
    app = QApplication(sys.argv)

    # Apply the custom stylesheet
    with open("Geoo.qss", "r") as file:
        app.setStyleSheet(file.read())
    window = DiagnosticWindow()
    window.show()
    sys.exit(app.exec_())
