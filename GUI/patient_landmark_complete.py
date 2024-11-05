from PyQt5 import QtCore, QtWidgets
import pyttsx3
import serial  # 시리얼 통신 라이브러리
import time

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(800, 480)
        self.navigation_image = QtWidgets.QPushButton(Dialog)
        self.navigation_image.setGeometry(QtCore.QRect(0, 0, 71, 51))
        self.navigation_image.setStyleSheet("#navigation_image{border-image: url(:/home/home.png);}")
        self.navigation_image.setText("")
        self.navigation_image.setObjectName("navigation_image")
        self.navigation_text = QtWidgets.QLabel(Dialog)
        self.navigation_text.setGeometry(QtCore.QRect(70, 20, 64, 15))
        self.navigation_text.setObjectName("navigation_text")
        self.landmark_text1 = QtWidgets.QLabel(Dialog)
        self.landmark_text1.setGeometry(QtCore.QRect(190, 80, 431, 30))
        self.landmark_text1.setStyleSheet("#landmark_text1{font-size: 24px; font-family: \'Arial\';}")
        self.landmark_text1.setObjectName("landmark_text1")
        self.landmark_text2 = QtWidgets.QLabel(Dialog)
        self.landmark_text2.setGeometry(QtCore.QRect(220, 130, 450, 30))
        self.landmark_text2.setStyleSheet("#landmark_text2{font-size: 24px; font-family: \'Arial\';}")
        self.landmark_text2.setObjectName("landmark_text2")
        self.landmark_text3 = QtWidgets.QLabel(Dialog)
        self.landmark_text3.setGeometry(QtCore.QRect(250, 200, 241, 30))
        self.landmark_text3.setStyleSheet("#landmark_text3{font-size: 24px; font-family: \'Arial\';}")
        self.landmark_text3.setObjectName("landmark_text3")
        self.back_text = QtWidgets.QLabel(Dialog)
        self.back_text.setGeometry(QtCore.QRect(260, 430, 291, 16))
        self.back_text.setObjectName("back_text")
        self.landmark_text4 = QtWidgets.QLabel(Dialog)
        self.landmark_text4.setGeometry(QtCore.QRect(150, 240, 500, 30))
        self.landmark_text4.setStyleSheet("#landmark_text4{font-size: 24px; font-family: \'Arial\';}")
        self.landmark_text4.setObjectName("landmark_text4")
        self.pushButton_lock = QtWidgets.QPushButton(Dialog)
        self.pushButton_lock.setGeometry(QtCore.QRect(280, 300, 200, 50))  # 버튼 위치 설정
        self.pushButton_lock.setStyleSheet("QPushButton{font-size: 24px; font-family: \'Arial\';}\n"
                                           "QPushButton:hover {\n"
                                           "background-color: skyblue;\n"
                                           "}")
        self.pushButton_lock.setObjectName("pushButton_lock")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.navigation_text.setText(_translate("Dialog", "길찾기"))
        self.landmark_text1.setText(_translate("Dialog", "얼굴 랜드마크 추출이 완료되었습니다."))
        self.landmark_text2.setText(_translate("Dialog", "신분증을 서랍에 넣어주시기 바랍니다."))
        self.landmark_text3.setText(_translate("Dialog", " 신분증을 찾으실 때는"))
        self.back_text.setText(_translate("Dialog", "20초 뒤 자동으로 홈화면으로 돌아갑니다."))
        self.landmark_text4.setText(_translate("Dialog", "홈화면의 \"신분증 찾기\" 버튼을 클릭해주세요."))
        self.pushButton_lock.setText(_translate("Dialog", "신분증 잠금"))

class landmarkCompleteWidget(QtWidgets.QWidget, Ui_Dialog):
    def __init__(self, parent=None):
        super(landmarkCompleteWidget, self).__init__(parent)
        self.setupUi(self)
        self.voice_prompt_executed = False  # 음성 안내 실행 여부 확인
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # 시리얼 포트를 클래스 속성으로 저장
            time.sleep(2)  # 시리얼 통신이 안정화될 때까지 대기
            print("Serial port opened successfully.")
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            self.ser = None  # 시리얼 포트를 열지 못했을 경우 None으로 설정

        # 버튼 클릭에만 홈 화면으로 전환되도록 설정
        self.pushButton_lock.clicked.connect(self.on_lock_button_clicked)

    def showEvent(self, event):
        super(landmarkCompleteWidget, self).showEvent(event)
        if not self.voice_prompt_executed:
            QtCore.QTimer.singleShot(500, self.start_voice_prompt)  # GUI가 완전히 렌더링된 후에 실행
            self.voice_prompt_executed = True  # 처음 실행 후 플래그 설정

    def start_voice_prompt(self):
        engine = pyttsx3.init()
        text_to_speak = f"{self.landmark_text1.text()}{self.landmark_text2.text()} {self.landmark_text3.text()}{self.landmark_text4.text()}"
        engine.say(text_to_speak)
        engine.runAndWait()

    def on_lock_button_clicked(self):
        self.rotate_servo()  # 서보모터 회전 명령
        self.return_to_home()

    def rotate_servo(self):
        if self.ser:
            self.ser.write(b'U')  # 아두이노로 'U' 명령 전송
            print("서보모터가 0도로 회전되었습니다.")
        else:
            print("Serial port not available.")

    def return_to_home(self):
        # 홈 화면으로 돌아가는 동작은 오직 버튼 클릭으로만 수행
        self.parent().setCurrentIndex(0)
        self.voice_prompt_executed = False  # 홈으로 돌아가면 플래그 초기화

    def closeEvent(self, event):
        if self.ser:
            self.ser.close()
            print("Serial port closed.")

import home_rc
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

