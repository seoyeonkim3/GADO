# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget
import pyttsx3
import home_rc
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(797, 480)
        Dialog.setStyleSheet("")
        self.map = QtWidgets.QLabel(Dialog)
        self.map.setGeometry(QtCore.QRect(120, 20, 541, 351))
        self.map.setStyleSheet("#map{border-image:url(:/map/injection.png)}")
        self.map.setText("")
        self.map.setObjectName("map")

        # map_text2 and map_text3 directly set with setGeometry
        self.map_text2 = QtWidgets.QLabel(Dialog)
        self.map_text2.setGeometry(QtCore.QRect(130, 360, 600, 30))  # Adjust the size and position as needed
        self.map_text2.setObjectName("map_text2")

        self.map_text3 = QtWidgets.QLabel(Dialog)
        self.map_text3.setGeometry(QtCore.QRect(130, 380, 600, 30))  # Adjust the size and position as needed
        self.map_text3.setObjectName("map_text3")

        self.map_text1 = QtWidgets.QLabel(Dialog)
        self.map_text1.setGeometry(QtCore.QRect(100, 10, 71, 31))  # Adjusted size for map_text1
        self.map_text1.setStyleSheet(
            "#map_text1{font-size: 20px; font-family: 'Arial'; color:red; border: 1px solid red;}")
        self.map_text1.setObjectName("map_text1")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.map_text2.setText(_translate("Dialog", "현재 위치에서 직진한 다음 오른쪽의 벽을 마주보며 접수 수납 구역을 지나갑니다."))
        self.map_text3.setText(_translate("Dialog", "접수 수납 구역을 지나 다시 한 번 오른쪽으로 꺾으면 왼편에 주사실이 위치해 있습니다."))
        self.map_text1.setText(_translate("Dialog", "주사실"))


class MapWidget(QWidget, Ui_Dialog):
    def __init__(self, parent=None):
        super(MapWidget, self).__init__(parent)
        self.setupUi(self)
        self.voice_prompt_executed = False  # Flag to check if voice prompt has been executed

        # 20초 후에 홈 화면으로 돌아가는 타이머 설정
        QtCore.QTimer.singleShot(20000, self.return_to_home)

    def showEvent(self, event):
        super(MapWidget, self).showEvent(event)
        if not self.voice_prompt_executed:
            QtCore.QTimer.singleShot(500, self.start_voice_prompt)  # Delay to ensure GUI is fully rendered
            self.voice_prompt_executed = True  # Set flag to True after first execution

    def start_voice_prompt(self):
        engine = pyttsx3.init()
        text_to_speak = f"{self.map_text2.text()} {self.map_text3.text()}"
        engine.say(text_to_speak)
        engine.runAndWait()

    def return_to_home(self):
        self.parent().setCurrentIndex(0)
        self.voice_prompt_executed = False  # Reset flag when returning to home
        
        
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # IdentificationCompleteWidget을 포함하는 메인 윈도우 생성
    main_window = QMainWindow()
    widget = MapWidget()
    main_window.setCentralWidget(widget)

    # 창 크기 설정 및 창 표시
    main_window.resize(800, 480)
    main_window.show()

    # 이벤트 루프 시작
    sys.exit(app.exec_())  
   
