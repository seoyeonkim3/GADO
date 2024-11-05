from PyQt5 import QtCore, QtWidgets
import pyttsx3
import threading
import pyaudio
import wave
import speech_recognition as sr
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWidgets import QWidget

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(800, 480)
        Dialog.setStyleSheet("QMainWindow { background-color: #F0F0F0; }")

        self.identification_text = QtWidgets.QLabel(Dialog)
        self.identification_text.setGeometry(QtCore.QRect(90, 20, 84, 15))
        self.identification_text.setObjectName("identification_text")

        self.medical_text = QtWidgets.QLabel(Dialog)
        self.medical_text.setGeometry(QtCore.QRect(740, 10, 51, 20))
        self.medical_text.setStyleSheet("#medical_text{border: 1px solid #000000;}")
        self.medical_text.setObjectName("medical_text")

        self.face_text1 = QtWidgets.QLabel(Dialog)
        self.face_text1.setGeometry(QtCore.QRect(240, 20, 411, 71))
        self.face_text1.setStyleSheet("#face_text1{font-size: 24px; font-family: 'Arial';}")
        self.face_text1.setObjectName("face_text1")

        self.record_image = QtWidgets.QLabel(Dialog)
        self.record_image.setGeometry(QtCore.QRect(270, 210, 211, 201))
        self.record_image.setStyleSheet("#record_image{border-image: url('/home/pi/myproject/myenv/PyQT_final/record.png');}")
        self.record_image.setText("")
        self.record_image.setObjectName("record_image")

        self.face_text2 = QtWidgets.QLabel(Dialog)
        self.face_text2.setGeometry(QtCore.QRect(220, 120, 601, 71))
        self.face_text2.setStyleSheet("#face_text2{font-size: 24px; font-family: 'Arial';}")
        self.face_text2.setObjectName("face_text2")

        self.record_text3 = QtWidgets.QLabel(Dialog)
        self.record_text3.setGeometry(QtCore.QRect(360, 20, 247, 161))
        self.record_text3.setStyleSheet("#record_text3{font-size: 24px; font-family: 'Arial';}")
        self.record_text3.setObjectName("record_text3")

        self.navigation_image = QtWidgets.QPushButton(Dialog)
        self.navigation_image.setGeometry(QtCore.QRect(0, 0, 93, 71))
        self.navigation_image.setStyleSheet("#navigation_image{border-image: url('/home/pi/myproject/myenv/PyQT_final/home.png');}")
        self.navigation_image.setText("")
        self.navigation_image.setObjectName("navigation_image")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.identification_text.setText(_translate("Dialog", "배송지 확인"))
        self.medical_text.setText(_translate("Dialog", "환자"))
        self.face_text1.setText(_translate("Dialog", "배송 의약품이 확인되었습니다."))
        self.face_text2.setText(_translate("Dialog", "배송지를 말씀해주세요."))
        self.record_text3.setText(_translate("Dialog", ". . ."))

class RecordHandler:
    def __init__(self, parent=None):
        self.parent = parent

    def start_voice_prompt(self):
        try:
            engine = pyttsx3.init()
            PROMPT1 = "배송 의약품이 확인되었습니다."
            PROMPT2 = "배송지를 말씀해주세요."
            engine.say(PROMPT1)
            engine.say(PROMPT2)
            engine.runAndWait()
        except Exception as e:
            print(f"Error in text-to-speech: {e}")

    def start_recording(self):
        threading.Thread(target=self.record_audio_and_process).start()

    def record_audio_and_process(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100
        CHUNK = 1024
        WAVE_OUTPUT_FILENAME = "output.wav"

        TARGET_WORDS = ["주사실"]

        def record_audio():
            audio = pyaudio.PyAudio()
            stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
            frames = []

            for _ in range(0, int(RATE / CHUNK * 5)):
                data = stream.read(CHUNK)
                frames.append(data)

            stream.stop_stream()
            stream.close()
            audio.terminate()

            waveFile = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
            waveFile.setnchannels(CHANNELS)
            waveFile.setsampwidth(audio.get_sample_size(FORMAT))
            waveFile.setframerate(RATE)
            waveFile.writeframes(b''.join(frames))
            waveFile.close()

        def recognize_speech():
            recognizer = sr.Recognizer()
            with sr.AudioFile(WAVE_OUTPUT_FILENAME) as source:
                audio = recognizer.record(source)

            try:
                recognized_text = recognizer.recognize_google(audio, language="ko-KR")
                print(f"Recognized Text: {recognized_text}")
            except sr.UnknownValueError:
                print("Google Web Speech API could not understand the audio")
                recognized_text = ""
            except sr.RequestError as e:
                print(f"Could not request results from Google Web Speech API; {e}")
                recognized_text = ""

            return recognized_text

        def preprocess_text(text):
            return text.lower().strip()

        record_audio()
        recognized_text = recognize_speech()

        recognized_text = preprocess_text(recognized_text)

        if any(word in recognized_text for word in TARGET_WORDS):
            self.parent.recording_complete.emit("주사실")
        else:
            self.parent.recording_complete.emit("not recognized")

class DestinationWidget(QtWidgets.QWidget, Ui_Dialog):
    recording_complete = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super(DestinationWidget, self).__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.record_handler = RecordHandler(self)

    def showEvent(self, event):
        super(DestinationWidget, self).showEvent(event)
        QtCore.QTimer.singleShot(500, self.record_handler.start_voice_prompt)
        self.ui.record_image.mousePressEvent = lambda event: self.record_handler.start_recording()
        
        
# 메인 코드
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # DestinationWidget을 포함하는 메인 윈도우 생성
    main_window = QMainWindow()
    widget = DestinationWidget()
    main_window.setCentralWidget(widget)

    # 창 크기 설정 및 창 표시
    main_window.resize(800, 480)
    main_window.show()

    # 이벤트 루프 시작
    sys.exit(app.exec_()) 
