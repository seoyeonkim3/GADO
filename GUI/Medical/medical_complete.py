from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QWidget
import pyttsx3
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QWidget

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(800, 480)
        Dialog.setStyleSheet("QMainWindow { background-color: #F0F0F0; }")

        self.deliver_text = QtWidgets.QLabel(Dialog)
        self.deliver_text.setGeometry(QtCore.QRect(90, 30, 100, 15))
        self.deliver_text.setObjectName("deliver_text")

        self.medical_text = QtWidgets.QLabel(Dialog)
        self.medical_text.setGeometry(QtCore.QRect(740, 10, 51, 20))
        self.medical_text.setStyleSheet("#medical_text{border: 1px solid #000000;}")
        self.medical_text.setObjectName("medical_text")

        self.deliver_text1 = QtWidgets.QLabel(Dialog)
        self.deliver_text1.setGeometry(QtCore.QRect(240, 20, 311, 71))
        self.deliver_text1.setStyleSheet("#deliver_text1{font-size: 24px; font-family: 'Arial';}")
        self.deliver_text1.setObjectName("deliver_text1")

        self.deliver_text2 = QtWidgets.QLabel(Dialog)
        self.deliver_text2.setGeometry(QtCore.QRect(220, 120, 401, 71))
        self.deliver_text2.setStyleSheet("#deliver_text2{font-size: 24px; font-family: 'Arial';}")
        self.deliver_text2.setObjectName("deliver_text2")

        self.navigation_image = QtWidgets.QPushButton(Dialog)
        self.navigation_image.setGeometry(QtCore.QRect(0, 0, 93, 71))
        self.navigation_image.setStyleSheet("#navigation_image{border-image: url('/home/pi/myproject/myenv/PyQT_final/home.png');}")
        self.navigation_image.setText("")
        self.navigation_image.setObjectName("navigation_image")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "배송 확인"))
        self.deliver_text.setText(_translate("Dialog", "배송 확인"))
        self.medical_text.setText(_translate("Dialog", "환자"))
        self.deliver_text1.setText(_translate("Dialog", "배송 접수가 완료되었습니다."))
        self.deliver_text2.setText(_translate("Dialog", "이용해주셔서 감사합니다."))

class RecordHandler:
    def __init__(self, parent=None):
        self.parent = parent

    def start_voice_prompt(self):
        try:
            engine = pyttsx3.init()
            PROMPT1 = "배송 접수가 완료되었습니다."
            PROMPT2 = "이용해주셔서 감사합니다."
            engine.say(PROMPT1)
            engine.say(PROMPT2)
            engine.runAndWait()
            print("Voice prompt played successfully.")  # Log message
        except Exception as e:
            print(f"Error in text-to-speech: {e}")  # Error log

class CompleteWidget(QWidget):
    def __init__(self, parent=None):
        super(CompleteWidget, self).__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.record_handler = RecordHandler(self)

    def showEvent(self, event):
        super(CompleteWidget, self).showEvent(event)
        QtCore.QTimer.singleShot(500, self.record_handler.start_voice_prompt)
        QtCore.QTimer.singleShot(10000, self.return_to_home)  # Changed to trigger after 10 seconds

    def return_to_home(self):
        print("Returning to home screen...")  # Debugging log
        parent = self.parentWidget()
        if parent is not None:
            parent.setCurrentIndex(0)  # Ensure that this correctly points to the home screen
            
# 메인 코드
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # CompleteWidget을 포함하는 메인 윈도우 생성
    main_window = QMainWindow()
    widget = CompleteWidget()
    main_window.setCentralWidget(widget)

    # 창 크기 설정 및 창 표시
    main_window.resize(800, 480)
    main_window.show()

    # 이벤트 루프 시작
    sys.exit(app.exec_()) 
