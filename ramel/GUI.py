# Form implementation generated from reading ui file 'untitled.ui'
#
# Created by: PyQt6 UI code generator 6.5.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets
import subprocess
import threading

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(583, 400)
        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(parent=self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(20, 0, 551, 381))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_5 = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setFamily("Rockwell")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.verticalLayout.addWidget(self.label_5)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        spacerItem = QtWidgets.QSpacerItem(30, 20, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_3 = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setFamily("Rockwell")
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_2.addWidget(self.label_3)
        self.num_robots = QtWidgets.QSpinBox(parent=self.verticalLayoutWidget)
        self.num_robots.setObjectName("num_robots")
        self.verticalLayout_2.addWidget(self.num_robots)
        self.btn_sim = QtWidgets.QPushButton(parent=self.verticalLayoutWidget)
        self.btn_sim.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.SizeVerCursor))
        self.btn_sim.setObjectName("btn_sim")
        self.verticalLayout_2.addWidget(self.btn_sim)
        spacerItem1 = QtWidgets.QSpacerItem(20, 50, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Fixed)
        self.verticalLayout_2.addItem(spacerItem1)
        self.horizontalLayout_4.addLayout(self.verticalLayout_2)
        spacerItem2 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem2)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setFamily("Rockwell")
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout_3.addWidget(self.label)
        self.num_cam = QtWidgets.QSpinBox(parent=self.verticalLayoutWidget)
        self.num_cam.setObjectName("num_cam")
        self.verticalLayout_3.addWidget(self.num_cam)
        self.btn_framework = QtWidgets.QPushButton(parent=self.verticalLayoutWidget)
        self.btn_framework.setObjectName("btn_framework")
        self.verticalLayout_3.addWidget(self.btn_framework)
        spacerItem3 = QtWidgets.QSpacerItem(20, 50, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Fixed)
        self.verticalLayout_3.addItem(spacerItem3)
        self.horizontalLayout_4.addLayout(self.verticalLayout_3)
        spacerItem4 = QtWidgets.QSpacerItem(30, 20, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem4)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_4 = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setFamily("Rockwell")
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_3.addWidget(self.label_4)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem5 = QtWidgets.QSpacerItem(60, 20, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout.addItem(spacerItem5)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_6 = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setFamily("Rockwell")
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label_6.setFont(font)
        self.label_6.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_4.addWidget(self.label_6)
        self.cb_recogida = QtWidgets.QComboBox(parent=self.verticalLayoutWidget)
        self.cb_recogida.setObjectName("cb_recogida")
        self.verticalLayout_4.addWidget(self.cb_recogida)
        spacerItem6 = QtWidgets.QSpacerItem(20, 50, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Fixed)
        self.verticalLayout_4.addItem(spacerItem6)
        self.horizontalLayout.addLayout(self.verticalLayout_4)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_7 = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setFamily("Rockwell")
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label_7.setFont(font)
        self.label_7.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.verticalLayout_6.addWidget(self.label_7)
        self.cb_entrega = QtWidgets.QComboBox(parent=self.verticalLayoutWidget)
        self.cb_entrega.setObjectName("cb_entrega")
        self.verticalLayout_6.addWidget(self.cb_entrega)
        spacerItem7 = QtWidgets.QSpacerItem(20, 50, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Fixed)
        self.verticalLayout_6.addItem(spacerItem7)
        self.horizontalLayout.addLayout(self.verticalLayout_6)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_8 = QtWidgets.QLabel(parent=self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setFamily("Rockwell")
        font.setPointSize(14)
        font.setBold(False)
        font.setWeight(50)
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.verticalLayout_5.addWidget(self.label_8)
        self.cb_robot = QtWidgets.QComboBox(parent=self.verticalLayoutWidget)
        self.cb_robot.setObjectName("cb_robot")
        self.verticalLayout_5.addWidget(self.cb_robot)
        spacerItem8 = QtWidgets.QSpacerItem(20, 50, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Fixed)
        self.verticalLayout_5.addItem(spacerItem8)
        self.horizontalLayout.addLayout(self.verticalLayout_5)
        spacerItem9 = QtWidgets.QSpacerItem(60, 20, QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Minimum)
        self.horizontalLayout.addItem(spacerItem9)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.btn_nav = QtWidgets.QPushButton(parent=self.verticalLayoutWidget)
        self.btn_nav.setObjectName("btn_nav")
        self.horizontalLayout_2.addWidget(self.btn_nav)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
        # Connect button signals to corresponding slots
        self.btn_sim.clicked.connect(self.on_btn_sim_clicked)
        self.btn_framework.clicked.connect(self.on_btn_framework_clicked)
        self.btn_nav.clicked.connect(self.on_btn_nav_clicked)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_5.setText(_translate("MainWindow", "Framework MultiRobot"))
        self.label_3.setText(_translate("MainWindow", "Número de robots:"))
        self.btn_sim.setText(_translate("MainWindow", "Simular"))
        self.label.setText(_translate("MainWindow", "Número de cámaras:"))
        self.btn_framework.setText(_translate("MainWindow", "Framework"))
        self.label_4.setText(_translate("MainWindow", "Distribuidor de Tareas"))
        self.label_6.setText(_translate("MainWindow", "Recogida"))
        self.label_7.setText(_translate("MainWindow", "Entrega"))
        self.label_8.setText(_translate("MainWindow", "Robot"))
        self.btn_nav.setText(_translate("MainWindow", "Navegar"))
    
    # Slot for the "Simular" button
    def on_btn_sim_clicked(self):
        # Run the ROS2 launch file for simulation in a separate thread
        sim_thread = threading.Thread(target=self.launch_simulation)
        sim_thread.start()

    def launch_simulation(self):
        subprocess.call(["ros2", "launch", "ramel", "ramel_mrs_sim.launch.py"])

    # Slot for the "Framework" button
    def on_btn_framework_clicked(self):
        # Get the selected number of robots and cameras from the spin boxes
        num_robots = self.num_robots.value()
        num_cameras = self.num_cam.value()

        # Run the ROS2 launch file for the framework with arguments in a separate thread
        framework_thread = threading.Thread(target=self.launch_framework, args=(num_robots, num_cameras))
        framework_thread.start()

    def launch_framework(self, num_robots, num_cameras):
        subprocess.call(["ros2", "launch", "ramel", "MRS.launch.py",
                         "n:={}".format(num_robots),
                         #"num_cam:={}".format(num_cameras)
                         ])

    # Slot for the "Navegar" button
    def on_btn_nav_clicked(self):
        # Get the selected values from the combo boxes
        recogida = self.cb_recogida.currentText()
        entrega = self.cb_entrega.currentText()
        robot = self.cb_robot.currentText()

        # Execute the Python file with parameters from combo boxes in a separate thread
        nav_thread = threading.Thread(target=self.execute_navigate, args=(recogida, entrega, robot))
        nav_thread.start()

    def execute_navigate(self, recogida, entrega, robot):
        subprocess.call(["python", "navigate.py", "--recogida", recogida, "--entrega", entrega, "--robot", robot])

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec())
