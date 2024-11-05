from PyQt5 import QtCore, QtGui, QtWidgets
import cv2
import mediapipe as mp
import csv
import time

class CameraThread(QtCore.QThread):
    change_pixmap_signal = QtCore.pyqtSignal(QtGui.QImage)
    recognition_complete = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.running = False
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_face_mesh = mp.solutions.face_mesh
        self.ellipse_center = (320, 240)
        self.axes_length = (100, 130)
        self.start_time = None
        self.recording_time = 3

    def run(self):
        self.running = True
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("웹캠을 열 수 없습니다.")
            self.running = False
            return

        with self.mp_face_mesh.FaceMesh(
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as face_mesh:

            with open('landmarks_patient.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['frame', 'landmark', 'x', 'y', 'z'])

                self.frame_count = 0

                while self.running and self.cap.isOpened():
                    success, image = self.cap.read()
                    if not success:
                        print("Ignoring empty camera frame.")
                        continue

                    image.flags.writeable = False
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    results = face_mesh.process(image)

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
                                    for idx, landmark in enumerate(face_landmarks.landmark):
                                        writer.writerow([self.frame_count, idx, landmark.x, landmark.y, landmark.z])
                                    self.cap.release()
                                    cv2.destroyAllWindows()
                                    self.running = False
                                    self.recognition_complete.emit()
                                    return
                            else:
                                self.start_time = None

                            self.mp_drawing.draw_landmarks(
                                image=image,
                                landmark_list=face_landmarks,
                                connections=self.mp_face_mesh.FACEMESH_TESSELATION,
                                landmark_drawing_spec=None,
                                connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=1, circle_radius=1))

                    cv2.ellipse(image, self.ellipse_center, self.axes_length, 0, 0, 360, (0, 255, 0), 2)
                    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb_image.shape
                    bytes_per_line = ch * w
                    qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
                    qt_image = qt_image.scaled(640, 480, QtCore.Qt.KeepAspectRatio)
                    self.change_pixmap_signal.emit(qt_image)

                    self.frame_count += 1

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


class Ui_direct_3(object):
    def setupUi(self, direct_3):
        direct_3.setObjectName("direct_3")
        direct_3.resize(800, 480)
        self.medical_text = QtWidgets.QLabel(direct_3)
        self.medical_text.setGeometry(QtCore.QRect(740, 10, 51, 20))
        self.medical_text.setStyleSheet("#medical_text{border: 1px solid #000000;}")
        self.medical_text.setObjectName("medical_text")
        self.navigation_image = QtWidgets.QLabel(direct_3)
        self.navigation_image.setGeometry(QtCore.QRect(0, 10, 71, 51))
        self.navigation_image.setStyleSheet("#navigation_image{border-image: url(:/home/home.png) 0 0 0 0 stretch stretch}")
        self.navigation_image.setText("")
        self.navigation_image.setObjectName("navigation_image")
        self.navigation_text = QtWidgets.QLabel(direct_3)
        self.navigation_text.setGeometry(QtCore.QRect(70, 20, 64, 15))
        self.navigation_text.setObjectName("navigation_text")
        self.face_text1 = QtWidgets.QLabel(direct_3)
        self.face_text1.setGeometry(QtCore.QRect(290, 10, 271, 71))
        self.face_text1.setStyleSheet("#face_text1{font-size: 24px; font-family: 'Arial';}")
        self.face_text1.setObjectName("face_text1")
        self.face_text2 = QtWidgets.QLabel(direct_3)
        self.face_text2.setGeometry(QtCore.QRect(260, 50, 321, 61))
        self.face_text2.setStyleSheet("#face_text2{font-size: 24px; font-family: 'Arial';}")
        self.face_text2.setObjectName("face_text2")
        self.direct_1 = QtWidgets.QLabel(direct_3)
        self.direct_1.setGeometry(QtCore.QRect(610, 110, 91, 91))
        self.direct_1.setStyleSheet("#direct_1{border-image:url(:/direction/direction_1.png);}")
        self.direct_1.setText("")
        self.direct_1.setObjectName("direct_1")
        self.direct_4 = QtWidgets.QLabel(direct_3)
        self.direct_4.setGeometry(QtCore.QRect(90, 330, 91, 91))
        self.direct_4.setStyleSheet("#direct_4{border-image:url(:/direction/direction_4.png);}")
        self.direct_4.setText("")
        self.direct_4.setObjectName("direct_4")
        self.direct_2 = QtWidgets.QLabel(direct_3)
        self.direct_2.setGeometry(QtCore.QRect(90, 110, 91, 91))
        self.direct_2.setStyleSheet("#direct_2{border-image:url(:/direction/direction_2.png);}")
        self.direct_2.setText("")
        self.direct_2.setObjectName("direct_2")
        self.direct_5 = QtWidgets.QLabel(direct_3)
        self.direct_5.setGeometry(QtCore.QRect(610, 330, 91, 91))
        self.direct_5.setStyleSheet("#direct_5{border-image:url(:/direction/direction_3.png);}")
        self.direct_5.setText("")
        self.direct_5.setObjectName("direct_5")
        self.warning_text1 = QtWidgets.QLabel(direct_3)
        self.warning_text1.setGeometry(QtCore.QRect(180, 430, 501, 20))
        self.warning_text1.setStyleSheet("#warning_text1{color:red}")
        self.warning_text1.setObjectName("warning_text1")
        self.warning_text2 = QtWidgets.QLabel(direct_3)
        self.warning_text2.setGeometry(QtCore.QRect(280, 450, 251, 20))
        self.warning_text2.setStyleSheet("#warning_text2{color:red}")
        self.warning_text2.setObjectName("warning_text2")
        self.camera = QtWidgets.QLabel(direct_3)
        self.camera.setGeometry(QtCore.QRect(109, 130, 571, 275))
        self.camera.setStyleSheet("background-color: black;")
        self.camera.setAlignment(QtCore.Qt.AlignCenter)
        self.camera.setObjectName("camera")

        self.retranslateUi(direct_3)
        QtCore.QMetaObject.connectSlotsByName(direct_3)

        self.thread = CameraThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.recognition_complete.connect(self.recognition_complete)
        self.thread.start()

    def retranslateUi(self, direct_3):
        _translate = QtCore.QCoreApplication.translate
        direct_3.setWindowTitle(_translate("direct_3", "Dialog"))
        self.medical_text.setText(_translate("direct_3", "의료진"))
        self.navigation_text.setText(_translate("direct_3", "길찾기"))
        self.face_text1.setText(_translate("direct_3", "신원 확인을 위한"))
        self.face_text2.setText(_translate("direct_3", "얼굴 인식을 시작합니다"))
        self.warning_text1.setText(_translate("direct_3", "주의사항: 카메라의 직사각형에 얼굴 전체가 들어갈 수 있게 조정해주세요."))
        self.warning_text2.setText(_translate("direct_3", "또, 촬영시 움직임을 최소화해주세요."))

    def update_image(self, image):
        self.camera.setPixmap(QtGui.QPixmap.fromImage(image))

    def recognition_complete(self):
        self.parentWidget().setCurrentIndex(6)  # Home widget으로 전환

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_direct_3()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
