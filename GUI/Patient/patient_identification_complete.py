from PyQt5 import QtCore, QtWidgets
import serial  # 시리얼 통신을 위해 추가
import time

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(800, 480)

        # 중앙에 "신분증 꺼내기" 버튼 추가
        self.pushButton_find = QtWidgets.QPushButton(Dialog)
        self.pushButton_find.setGeometry(QtCore.QRect(280, 280, 200, 50))  # 위치 (x, y)와 크기 (width, height)
        self.pushButton_find.setStyleSheet("QPushButton{font-size: 24px; font-family: \'Arial\';}\n"
                                           "QPushButton:hover {\n"
                                           "background-color: skyblue;\n"
                                           "}")
        self.pushButton_find.setObjectName("pushButton_find")

        # 나머지 UI 요소들
        self.identification_text1 = QtWidgets.QLabel(Dialog)
        self.identification_text1.setGeometry(QtCore.QRect(260, 50, 261, 30))
        self.identification_text1.setStyleSheet("#identification_text1{font-size: 24px; font-family: \'Arial\';}")
        self.identification_text1.setObjectName("identification_text1")
        self.identification_text2 = QtWidgets.QLabel(Dialog)
        self.identification_text2.setGeometry(QtCore.QRect(120, 130, 611, 30))
        self.identification_text2.setStyleSheet("#identification_text2{font-size: 24px; font-family: \'Arial\';}")
        self.identification_text2.setObjectName("identification_text2")
        self.identification_text3 = QtWidgets.QLabel(Dialog)
        self.identification_text3.setGeometry(QtCore.QRect(250, 210, 291, 30))
        self.identification_text3.setStyleSheet("#identification_text3{font-size: 24px; font-family: \'Arial\';}")
        self.identification_text3.setObjectName("identification_text3")
        self.back_text = QtWidgets.QLabel(Dialog)
        self.back_text.setGeometry(QtCore.QRect(250, 430, 300, 15))
        self.back_text.setObjectName("back_text")
        self.navigation_image = QtWidgets.QPushButton(Dialog)
        self.navigation_image.setGeometry(QtCore.QRect(0, 0, 71, 51))
        self.navigation_image.setStyleSheet("#navigation_image{border-image: url(:/home/home.png);}")
        self.navigation_image.setText("")
        self.navigation_image.setObjectName("navigation_image")
        self.navigation_text = QtWidgets.QLabel(Dialog)
        self.navigation_text.setGeometry(QtCore.QRect(70, 20, 64, 15))
        self.navigation_text.setObjectName("navigation_text")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

        # 버튼 클릭 시 서보모터 회전 연결
        self.pushButton_find.clicked.connect(self.rotate_servo)

        # 시리얼 포트 설정
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            time.sleep(2)  # 시리얼 통신이 안정화될 때까지 대기
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            self.ser = None  # 시리얼 포트를 열지 못했을 경우 None으로 설정

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.pushButton_find.setText(_translate("Dialog", "신분증 꺼내기"))  # 버튼에 텍스트 추가
        self.identification_text1.setText(_translate("Dialog", "신원이 확인되었습니다."))
        self.identification_text2.setText(_translate("Dialog", "아래 신분증 꺼내기 버튼을 눌러 신분증을 찾아가주세요."))
        self.identification_text3.setText(_translate("Dialog", "이용해주셔서 감사합니다."))
        self.back_text.setText(_translate("Dialog", "20초 뒤 자동으로 홈화면으로 돌아갑니다."))
        self.navigation_text.setText(_translate("Dialog", "길찾기"))

    def rotate_servo(self):
        if self.ser:
            self.ser.write(b'U')  # 아두이노로 'L' 명령 전송하여 서보모터 90도 회전
            print("서보모터가 90도로 회전되었습니다.")
        else:
            print("Serial port not available.")

    def close_serial_port(self):
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

