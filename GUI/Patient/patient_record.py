# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget
import pyaudio
import wave
import speech_recognition as sr
import pyttsx3
import threading
from clickablelabel import ClickableLabel

class Ui_Dialog(object):
    recording_done = QtCore.pyqtSignal()

    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(800, 480)
        self.navigation_text = QtWidgets.QLabel(Dialog)
        self.navigation_text.setGeometry(QtCore.QRect(70, 20, 64, 15))
        self.navigation_text.setObjectName("navigation_text")
        self.navigation_image = QtWidgets.QLabel(Dialog)
        self.navigation_image.setGeometry(QtCore.QRect(0, 0, 71, 51))
        self.navigation_image.setStyleSheet(
            "#navigation_image{border-image: url(:/home/home.png) 0 0 0 0 stretch stretch}")
        self.navigation_image.setText("")
        self.navigation_image.setObjectName("navigation_image")
        self.record_image = ClickableLabel(Dialog)
        self.record_image.setGeometry(QtCore.QRect(270, 210, 211, 201))
        self.record_image.setStyleSheet("#record_image{border-image: url(:/record/record.png);}")
        self.record_image.setText("")
        self.record_image.setObjectName("record_image")
        self.medical_text = QtWidgets.QLabel(Dialog)
        self.medical_text.setGeometry(QtCore.QRect(740, 10, 51, 20))
        self.medical_text.setStyleSheet("#medical_text{border: 1px solid #000000;}")
        self.medical_text.setObjectName("medical_text")
        self.record_text2 = QtWidgets.QLabel(Dialog)
        self.record_text2.setGeometry(QtCore.QRect(260, 80, 247, 51))
        self.record_text2.setStyleSheet("#record_text2{font-size: 24px; font-family: 'Arial';}")
        self.record_text2.setObjectName("record_text2")
        self.record_text1 = QtWidgets.QLabel(Dialog)
        self.record_text1.setGeometry(QtCore.QRect(270, 20, 247, 81))
        self.record_text1.setStyleSheet("#record_text1{font-size: 24px; font-family: 'Arial';}")
        self.record_text1.setObjectName("record_text1")
        self.record_text3 = QtWidgets.QLabel(Dialog)
        self.record_text3.setGeometry(QtCore.QRect(350, 80, 247, 161))
        self.record_text3.setStyleSheet("#record_text3{font-size: 24px; font-family: 'Arial';}")
        self.record_text3.setObjectName("record_text3")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.navigation_text.setText(_translate("Dialog", "길찾기"))
        self.medical_text.setText(_translate("Dialog", "의료진"))
        self.record_text2.setText(_translate("Dialog", "목적지를 말씀해주세요"))
        self.record_text1.setText(_translate("Dialog", "아래 음성인식을 눌러"))
        self.record_text3.setText(_translate("Dialog", ". . ."))

class RecordWidget(QWidget, Ui_Dialog):
    def __init__(self, parent=None):
        super(RecordWidget, self).__init__(parent)
        self.setupUi(self)  # Initialize the UI

    def showEvent(self, event):
        super(RecordWidget, self).showEvent(event)
        QtCore.QTimer.singleShot(500, self.start_voice_prompt_and_recognition)

    def start_voice_prompt_and_recognition(self):
        # 음성 안내 실행
        try:
            engine = pyttsx3.init()
            PROMPT = "아래 음성인식을 눌러 목적지를 말씀해주세요."
            engine.say(PROMPT)
            engine.runAndWait()
        except Exception as e:
            print(f"Error in text-to-speech: {e}")

        # 음성 인식 시작
        threading.Thread(target=self.record_audio_and_process).start()

    def record_audio_and_process(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100
        CHUNK = 1024
        WAVE_OUTPUT_FILENAME = "output.wav"

        TARGET_WORDS = ["접수", "수납", "약국", "의약품 저장고", "주사실", "X선 촬영실", "영상의학과", "응급센터", "화장실", "엘리베이터", "출입구"]
        RESPONSE_YES = "길 안내 기능을 제공해드리겠습니다."
        RESPONSE_NO = "길찾기 기능을 제공할 수 없습니다."

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

        # 음성 녹음 및 인식
        record_audio()
        recognized_text = recognize_speech()

        # 인식된 텍스트 후처리
        recognized_text = preprocess_text(recognized_text)

        engine = pyttsx3.init()

        # 인식된 텍스트에 따라 응답
        if any(preprocess_text(word) in recognized_text for word in TARGET_WORDS):
            print(RESPONSE_YES)
            engine.say(RESPONSE_YES)
            engine.runAndWait()
            QTimer.singleShot(0, lambda: self.parentWidget().parentWidget().setCurrentIndex(3))
        else:
            print(RESPONSE_NO)
            engine.say(RESPONSE_NO)
            engine.runAndWait()
            QTimer.singleShot(0, lambda: self.parentWidget().parentWidget().setCurrentIndex(0))
