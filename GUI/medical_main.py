import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QWidget, QMessageBox
from PyQt5.QtCore import pyqtSlot, QTimer, QMetaObject, Qt, Q_ARG
from PyQt5 import QtCore, QtGui
from medical_home import Ui_Dialog as HomeUi
from medical_face_detection import Ui_Dialog as FaceDetectionUi
from medical_identification_complete import IdentificationCompleteWidget
from medical_identification_complete_destination import DestinationWidget
from medical_deliver import DeliverWidget
from medical_complete import CompleteWidget
from medical_face_detection import CameraThread
import logging
logging.getLogger('tensorflow').setLevel(logging.ERROR)

class HomeWidget(QWidget, HomeUi):
    def __init__(self, parent=None):
        super(HomeWidget, self).__init__(parent)
        self.setupUi(self)
        self.pushButton_2.clicked.connect(self.show_face_detection_widget)

    @pyqtSlot()
    def show_face_detection_widget(self):
        self.parentWidget().setCurrentIndex(1)

class FaceDetectionWidget(QWidget, FaceDetectionUi):
    def __init__(self, parent=None):
        super(FaceDetectionWidget, self).__init__(parent)
        self.setupUi(self)

        self.thread = CameraThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.recognition_complete.connect(self.handle_recognition_result)
        self.thread.running = True
        self.thread.start()

    def update_image(self, image):
        self.camera_label.setPixmap(QtGui.QPixmap.fromImage(image))

    @pyqtSlot(str)
    def handle_recognition_result(self, message):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText(message)
        msg_box.setWindowTitle("신원 확인 결과")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()

        if "신원이 확인되었습니다" in message:
            self.parentWidget().setCurrentIndex(2)

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Medical System")
        self.setGeometry(100, 100, 800, 480)

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.home_widget = HomeWidget(self)
        self.face_detection_widget = FaceDetectionWidget(self)
        self.identification_complete_widget = IdentificationCompleteWidget(self)
        self.destination_widget = DestinationWidget(self)
        self.deliver_widget = DeliverWidget(self)
        self.complete_widget = CompleteWidget(self)

        self.stack.addWidget(self.home_widget)
        self.stack.addWidget(self.face_detection_widget)
        self.stack.addWidget(self.identification_complete_widget)
        self.stack.addWidget(self.destination_widget)
        self.stack.addWidget(self.deliver_widget)
        self.stack.addWidget(self.complete_widget)
        self.stack.setCurrentIndex(0)  # Make sure to start with the home screen

        self.identification_complete_widget.recording_complete.connect(self.handle_navigation)
        self.destination_widget.recording_complete.connect(self.handle_navigation)
        self.deliver_widget.switch_to_medical_complete.connect(self.show_complete_widget)

    @pyqtSlot(str)
    def handle_navigation(self, result):
        if result == "recognized":
            self.stack.setCurrentIndex(3)  # Switch to the Destination screen
        elif result == "주사실":
            self.stack.setCurrentIndex(4)  # Switch to the Deliver screen
        else:
            self.stack.setCurrentIndex(0)  # Return to the Home screen

    @pyqtSlot()
    def show_complete_widget(self):
        self.stack.setCurrentIndex(5)  # Switch to the Complete screen

    def closeEvent(self, event):
        self.face_detection_widget.closeEvent(event)
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

