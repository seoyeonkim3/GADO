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
        self.deliver_text.setGeometry(QtCore.QRect(90, 30, 131, 16))
        self.deliver_text.setObjectName("deliver_text")

        self.deliver_text1 = QtWidgets.QLabel(Dialog)
        self.deliver_text1.setGeometry(QtCore.QRect(50, 170, 389, 31))
        self.deliver_text1.setStyleSheet("#deliver_text1{font-size: 24px; font-family: 'Arial';}")
        self.deliver_text1.setObjectName("deliver_text1")

        self.deliver_text2 = QtWidgets.QLabel(Dialog)
        self.deliver_text2.setGeometry(QtCore.QRect(50, 310, 389, 51))
        self.deliver_text2.setStyleSheet("#deliver_text2{font-size: 24px; font-family: 'Arial';}")
        self.deliver_text2.setObjectName("deliver_text2")

        self.completion_button = QtWidgets.QPushButton(Dialog)
        self.completion_button.setGeometry(QtCore.QRect(680, 427, 93, 41))
        self.completion_button.setStyleSheet("QPushButton{border: 2px solid black;}\n"
                                             "#completion_image{font-size: 24px; font-family: 'Arial';}\n"
                                             "QPushButton:hover {\n"
                                             "background-color: skyblue;\n"
                                             "}")
        self.completion_button.setObjectName("completion_button")

        self.navigation_image = QtWidgets.QPushButton(Dialog)
        self.navigation_image.setGeometry(QtCore.QRect(0, 0, 93, 71))
        self.navigation_image.setStyleSheet("#navigation_image{border-image: url('/home/pi/myproject/myenv/PyQT_final/home.png');}")
        self.navigation_image.setText("")
        self.navigation_image.setObjectName("navigation_image")

        self.medical_text = QtWidgets.QLabel(Dialog)
        self.medical_text.setGeometry(QtCore.QRect(740, 10, 51, 20))
        self.medical_text.setStyleSheet("#medical_text{border: 1px solid black;}")
        self.medical_text.setObjectName("medical_text")

        self.deliver_announcement = QtWidgets.QLabel(Dialog)
        self.deliver_announcement.setGeometry(QtCore.QRect(180, 90, 511, 51))
        self.deliver_announcement.setStyleSheet("#deliver_announcement{font-size: 24px; font-family: 'Arial';}")
        self.deliver_announcement.setObjectName("deliver_announcement")

        self.comboBox = QtWidgets.QComboBox(Dialog)
        self.comboBox.setGeometry(QtCore.QRect(53, 230, 300, 50))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")

        self.comboBox_2 = QtWidgets.QComboBox(Dialog)
        self.comboBox_2.setGeometry(QtCore.QRect(50, 370, 300, 50))
        self.comboBox_2.setObjectName("comboBox_2")
        self.comboBox_2.addItem("")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.deliver_text.setText(_translate("Dialog", "의약품 배송 입력"))
        self.deliver_text1.setText(_translate("Dialog", "배송 대상 물품"))
        self.deliver_text2.setText(_translate("Dialog", "목적지"))
        self.completion_button.setText(_translate("Dialog", "완료"))
        self.medical_text.setText(_translate("Dialog", "환자"))
        self.deliver_announcement.setText(_translate("Dialog", "배송 대상 물품과 목적지를 확인해주십시오."))
        self.comboBox.setItemText(0, _translate("Dialog", "Needle"))
        self.comboBox.setItemText(1, _translate("Dialog", "Fluid"))
        self.comboBox.setItemText(2, _translate("Dialog", "Syringe"))
        self.comboBox.setItemText(3, _translate("Dialog", "Kelly"))
        self.comboBox.setItemText(4, _translate("Dialog", "Scissors"))
        self.comboBox.setItemText(5, _translate("Dialog", "Mess"))
        self.comboBox_2.setItemText(0, _translate("Dialog", "주사실"))


class RecordHandler:
    def __init__(self, parent=None):
        self.parent = parent

    def start_voice_prompt(self):
        try:
            print("Attempting to start TTS engine...")  # Debugging log
            engine = pyttsx3.init()
            PROMPT1 = "배송 대상 물품과 목적지를 확인해주십시오."
            engine.say(PROMPT1)
            engine.runAndWait()
            print("Voice prompt played successfully.")
        except Exception as e:
            print(f"Error in text-to-speech: {e}")


class DeliverWidget(QWidget):
    switch_to_medical_complete = QtCore.pyqtSignal()  # Signal moved to DeliverWidget

    def __init__(self, parent=None):
        super(DeliverWidget, self).__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.record_handler = RecordHandler(self)

        # Connect button click to signal emission
        self.ui.completion_button.clicked.connect(self.switch_to_medical_complete)

    def showEvent(self, event):
        super(DeliverWidget, self).showEvent(event)
        QtCore.QTimer.singleShot(500, self.record_handler.start_voice_prompt)
        
        
# 메인 코드
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # DeliverWidget을 포함하는 메인 윈도우 생성
    main_window = QMainWindow()
    widget = DeliverWidget()
    main_window.setCentralWidget(widget)

    # 창 크기 설정 및 창 표시
    main_window.resize(800, 480)
    main_window.show()

    # 이벤트 루프 시작
    sys.exit(app.exec_())  
   
