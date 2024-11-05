from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Dialog(object):
    switch_to_face_detection = QtCore.pyqtSignal()  # 시그널 정의

    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(800, 480)
        self.HOME_icon = QtWidgets.QLabel(Dialog)
        self.HOME_icon.setGeometry(QtCore.QRect(310, 0, 161, 131))
        self.HOME_icon.setStyleSheet("QLabel#HOME_icon {\n"
                                     "    border-image: url(:/home/home.png) 0 0 0 0 stretch stretch;\n"
                                     "}")
        self.HOME_icon.setText("")
        self.HOME_icon.setObjectName("HOME_icon")
        self.HOME_text = QtWidgets.QLabel(Dialog)
        self.HOME_text.setGeometry(QtCore.QRect(280, 140, 250, 61))
        self.HOME_text.setStyleSheet("#HOME_text{font-size: 30px; font-family: \\\'Arial\\\';}")
        self.HOME_text.setObjectName("HOME_text")
        self.horizontalLayoutWidget = QtWidgets.QWidget(Dialog)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(90, 260, 651, 211))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton.setStyleSheet("QPushButton{font-size: 24px; font-family: \'Arial\';}\n"
                                      "QPushButton:hover {\n"
                                      "background-color: skyblue;\n"
                                      "}")
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        self.pushButton_2.setStyleSheet("QPushButton{font-size: 24px; font-family: \'Arial\';}\n"
                                        "QPushButton:hover {\n"
                                        "background-color: skyblue;\n"
                                        "}")
        self.pushButton_2.setObjectName("pushButton_2")
        self.horizontalLayout.addWidget(self.pushButton_2)

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

        # 버튼 클릭 시 시그널 발송
        self.pushButton_2.clicked.connect(self.switch_to_face_detection.emit)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.HOME_text.setText(_translate("Dialog", "HOME (의료진용)"))
        self.pushButton.setText(_translate("Dialog", "기본 안내문 전달"))
        self.pushButton_2.setText(_translate("Dialog", "의약품 배송"))

import home_rc
