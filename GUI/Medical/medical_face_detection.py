import warnings
warnings.filterwarnings("ignore", category=UserWarning, module='google.protobuf.symbol_database')

from absl import logging
logging.set_verbosity(logging.ERROR)

from PyQt5 import QtCore, QtGui, QtWidgets
import cv2
import mediapipe as mp
import csv
import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont

class CameraThread(QtCore.QThread):
    change_pixmap_signal = QtCore.pyqtSignal(QtGui.QImage)
    recognition_complete = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = True
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_face_mesh = mp.solutions.face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.ellipse_center = (320, 240)
        self.axes_length = (100, 130)
        self.start_time = None
        self.recording_time = 3

    def run(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("cannot open camera")
            self.running = False
            return

        while self.running:
            ret, image = self.cap.read()
            if not ret:
                #print("Ignoring empty camera frame.")
                continue

            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.mp_face_mesh.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    face_center_x = int(face_landmarks.landmark[1].x * image.shape[1])
                    face_center_y = int(face_landmarks.landmark[1].y * image.shape[0])
                    face_center = (face_center_x, face_center_y)

                    if self.is_point_in_ellipse(face_center, self.ellipse_center, self.axes_length):
                        if self.start_time is None:
                            self.start_time = time.time()
                        elif time.time() - self.start_time >= self.recording_time:
                            current_landmarks = []
                            for landmark in face_landmarks.landmark:
                                current_landmarks.append([landmark.x, landmark.y, landmark.z])
                            current_landmarks = self.normalize_landmarks(current_landmarks)
                            current_landmarks = np.array(current_landmarks, dtype=np.float32).flatten()

                            # ¿©±â¼­ ½Å¿ø È®ÀÎ °ü·Ã Ã³¸®°¡ ÀÌ·ç¾îÁ®¾ß ÇÏÁö¸¸, FAISS¿Í À¯»çµµ ºñ±³ ÄÚµå¸¦ Á¦°ÅÇß½À´Ï´Ù.
                            message = "신원이 확인되었습니다"  # ´õ¹Ì ¸Þ½ÃÁö

                            self.recognition_complete.emit(message)
                            self.stop()
                            return
                    else:
                        self.start_time = None

                    self.mp_drawing.draw_landmarks(
                        image=image,
                        landmark_list=face_landmarks,
                        connections=mp.solutions.face_mesh.FACEMESH_TESSELATION,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1)
                    )

            else:
                self.start_time = None

            cv2.ellipse(image, self.ellipse_center, self.axes_length, 0, 0, 360, (0, 255, 0), 2)
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            qt_image = qt_image.scaled(640, 480, QtCore.Qt.KeepAspectRatio)
            self.change_pixmap_signal.emit(qt_image)

        self.cap.release()

    def is_point_in_ellipse(self, point, center, axes):
        px, py = point
        cx, cy = center
        rx, ry = axes
        return ((px - cx) ** 2) / rx ** 2 + ((py - cy) ** 2) / ry ** 2 <= 1

    def stop(self):
        self.running = False
        if self.cap.isOpened():
            self.cap.release()
        self.wait()

    def normalize_landmarks(self, landmarks):
        landmarks = np.array(landmarks)
        center = np.mean(landmarks, axis=0)
        return landmarks - center

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(800, 480)
        Dialog.setStyleSheet("#medical_text{border: 1px solid black;}")
        self.face_text2 = QtWidgets.QLabel(Dialog)
        self.face_text2.setGeometry(QtCore.QRect(260, 30, 321, 61))
        self.face_text2.setStyleSheet("#face_text2{font-size: 24px; font-family: 'Arial';}")
        self.face_text2.setObjectName("face_text2")
        self.face_text1 = QtWidgets.QLabel(Dialog)
        self.face_text1.setGeometry(QtCore.QRect(280, -10, 271, 71))
        self.face_text1.setStyleSheet("#face_text1{font-size: 24px; font-family: 'Arial';}")
        self.face_text1.setObjectName("face_text1")
        self.warning_text1 = QtWidgets.QLabel(Dialog)
        self.warning_text1.setGeometry(QtCore.QRect(180, 430, 501, 20))
        self.warning_text1.setStyleSheet("#warning_text1{color:red}")
        self.warning_text1.setObjectName("warning_text1")
        self.warning_text2 = QtWidgets.QLabel(Dialog)
        self.warning_text2.setGeometry(QtCore.QRect(280, 450, 251, 20))
        self.warning_text2.setStyleSheet("#warning_text2{color:red}")
        self.warning_text2.setObjectName("warning_text2")
        self.direct_2 = QtWidgets.QLabel(Dialog)
        self.direct_2.setGeometry(QtCore.QRect(90, 90, 91, 91))
        self.direct_2.setStyleSheet("#direct_2{border-image:url(:/direction/direction_2.png);}")
        self.direct_2.setText("")
        self.direct_2.setObjectName("direct_2")
        self.direct_1 = QtWidgets.QLabel(Dialog)
        self.direct_1.setGeometry(QtCore.QRect(610, 90, 91, 91))
        self.direct_1.setStyleSheet("#direct_1{border-image:url(:/direction/direction_1.png);}")
        self.direct_1.setText("")
        self.direct_1.setObjectName("direct_1")
        self.direct_4 = QtWidgets.QLabel(Dialog)
        self.direct_4.setGeometry(QtCore.QRect(90, 330, 91, 91))
        self.direct_4.setStyleSheet("#direct_4{border-image:url(:/direction/direction_4.png);}")
        self.direct_4.setText("")
        self.direct_4.setObjectName("direct_4")
        self.direct_5 = QtWidgets.QLabel(Dialog)
        self.direct_5.setGeometry(QtCore.QRect(610, 330, 91, 91))
        self.direct_5.setStyleSheet("#direct_5{border-image:url(:/direction/direction_3.png);}")
        self.direct_5.setText("")
        self.direct_5.setObjectName("direct_5")
        self.camera_label = QtWidgets.QLabel(Dialog)
        self.camera_label.setGeometry(QtCore.QRect(109, 130, 571, 275))
        self.camera_label.setStyleSheet("background-color: black;")
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setObjectName("camera_label")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

        self.thread = CameraThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.recognition_complete.connect(self.show_message)
        self.thread.start()

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.face_text2.setText(_translate("Dialog", "얼굴 인식을 시작합니다"))
        self.face_text1.setText(_translate("Dialog", "신원 확인을 위한"))
        self.warning_text1.setText(_translate("Dialog", "주의사항: 카메라의 직사각형에 얼굴 전체가 들어갈 수 있게 조정해주세요."))
        self.warning_text2.setText(_translate("Dialog", "또, 촬영 시  움직임을 최소화해주세요."))

    def update_image(self, image):
        self.camera_label.setPixmap(QtGui.QPixmap.fromImage(image))

    def show_message(self, message):
        msg_box = QtWidgets.QMessageBox()
        msg_box.setIcon(QtWidgets.QMessageBox.Information)
        msg_box.setText(message)
        msg_box.setWindowTitle("신원 확인 결과")
        msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
        msg_box.setWindowModality(QtCore.Qt.ApplicationModal)
        msg_box.exec_()

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_Dialog()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

