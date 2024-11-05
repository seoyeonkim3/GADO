import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QWidget, QMessageBox
from PyQt5.QtCore import pyqtSlot, pyqtSignal, QTimer, Qt, QMetaObject, Q_ARG
from PyQt5 import QtWidgets
from PyQt5.QtGui import QPixmap  # QPixmap 임포트
from patient_home import Ui_Dialog as HomeUi
from patient_webcam import Ui_direct_3 as WebcamUi
from patient_record import Ui_Dialog as RecordUi
from patient_map_pharmacy import Ui_Dialog as PharmacyMapUi
from patient_map_imaging import Ui_Dialog as ImagingMapUi
from patient_map_injection import Ui_Dialog as InjectionMapUi
from patient_landmark_complete import landmarkCompleteWidget
from patient_identification import Ui_Dialog_identification as IdentificationUi
from patient_identification import CameraThread
from patient_identification_complete import Ui_Dialog as IdentificationCompleteUi

import pyttsx3
import threading
import pyaudio
import wave
import speech_recognition as sr
import home_rc


class HomeWidget(QWidget, HomeUi):
    def __init__(self, parent=None):
        super(HomeWidget, self).__init__(parent)
        self.setupUi(self)
        self.pushButton_receipt.clicked.connect(self.show_webcam_widget)
        self.pushButton_navigation.clicked.connect(self.show_record_widget)
        self.pushButton_identification.clicked.connect(self.show_identification_widget)

    @pyqtSlot()
    def show_webcam_widget(self):
        self.parentWidget().setCurrentIndex(1)

    @pyqtSlot()
    def show_record_widget(self):
        self.parentWidget().setCurrentIndex(2)
        QTimer.singleShot(500, self.parentWidget().widget(2).play_prompt)

    @pyqtSlot()
    def show_identification_widget(self):
        print("Identification button clicked")  # 디버깅용 출력
        print("Current Index Before:", self.parentWidget().currentIndex())  # 현재 인덱스 확인
        self.parentWidget().setCurrentIndex(7)
        print("Current Index After:", self.parentWidget().currentIndex())

class WebcamWidget(QWidget, WebcamUi):
    def __init__(self, parent=None):
        super(WebcamWidget, self).__init__(parent)
        self.setupUi(self)

    @pyqtSlot()
    def show_home_widget(self):
        self.parentWidget().setCurrentIndex(0)


class RecordWidget(QWidget, RecordUi):
    def __init__(self, parent=None):
        super(RecordWidget, self).__init__(parent)
        self.setupUi(self)

        self.record_image.mousePressEvent = self.start_recording

    @pyqtSlot()
    def show_home_widget(self):
        self.parentWidget().setCurrentIndex(0)
    def play_prompt(self):
        engine = pyttsx3.init()
        PROMPT = "아래 음성인식을 눌러 목적지를 말씀해주세요."
        engine.say(PROMPT)
        engine.runAndWait()

    def start_recording(self, event):
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

        engine = pyttsx3.init()

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

        if any(preprocess_text(word) in recognized_text for word in TARGET_WORDS):
            print(RESPONSE_YES)
            engine.say(RESPONSE_YES)
            engine.runAndWait()
            if '약국' in recognized_text:
                QMetaObject.invokeMethod(self.parentWidget(), "setCurrentIndex", Qt.QueuedConnection, Q_ARG(int, 3))
            elif '영상의학과' in recognized_text:
                QMetaObject.invokeMethod(self.parentWidget(), "setCurrentIndex", Qt.QueuedConnection, Q_ARG(int, 4))
            elif '주사실' in recognized_text:
                QMetaObject.invokeMethod(self.parentWidget(), "setCurrentIndex", Qt.QueuedConnection, Q_ARG(int, 5))
            else:
                QMetaObject.invokeMethod(self.parentWidget(), "setCurrentIndex", Qt.QueuedConnection, Q_ARG(int, 0))
        else:
            print(RESPONSE_NO)
            engine.say(RESPONSE_NO)
            engine.runAndWait()
            QMetaObject.invokeMethod(self.parentWidget(), "setCurrentIndex", Qt.QueuedConnection, Q_ARG(int, 0))


class PharmacyMapWidget(QWidget, PharmacyMapUi):
    def __init__(self, parent=None):
        super(PharmacyMapWidget, self).__init__(parent)
        self.setupUi(self)
        self.voice_prompt_executed = False

        QTimer.singleShot(20000, self.return_to_home)

    def showEvent(self, event):
        super(PharmacyMapWidget, self).showEvent(event)
        if not self.voice_prompt_executed:
            QTimer.singleShot(500, self.start_voice_prompt)
            self.voice_prompt_executed = True

    def start_voice_prompt(self):
        engine = pyttsx3.init()
        text_to_speak = f"{self.map_text2.text()} {self.map_text3.text()}"
        engine.say(text_to_speak)
        engine.runAndWait()

    def return_to_home(self):
        QMetaObject.invokeMethod(self.parent(), "setCurrentIndex", Qt.QueuedConnection, Q_ARG(int, 0))
        self.voice_prompt_executed = False


class ImagingMapWidget(QWidget, ImagingMapUi):
    def __init__(self, parent=None):
        super(ImagingMapWidget, self).__init__(parent)
        self.setupUi(self)
        self.voice_prompt_executed = False

        QTimer.singleShot(20000, self.return_to_home)

    def showEvent(self, event):
        super(ImagingMapWidget, self).showEvent(event)
        if not self.voice_prompt_executed:
            QTimer.singleShot(500, self.start_voice_prompt)
            self.voice_prompt_executed = True

    def start_voice_prompt(self):
        engine = pyttsx3.init()
        text_to_speak = f"{self.map_text2.text()} {self.map_text3.text()} {self.map_text4.text()}"
        engine.say(text_to_speak)
        engine.runAndWait()

    def return_to_home(self):
        QMetaObject.invokeMethod(self.parent(), "setCurrentIndex", Qt.QueuedConnection, Q_ARG(int, 0))
        self.voice_prompt_executed = False


class InjectionMapWidget(QWidget, InjectionMapUi):
    def __init__(self, parent=None):
        super(InjectionMapWidget, self).__init__(parent)
        self.setupUi(self)
        self.voice_prompt_executed = False

        QTimer.singleShot(20000, self.return_to_home)

    def showEvent(self, event):
        super(InjectionMapWidget, self).showEvent(event)
        if not self.voice_prompt_executed:
            QTimer.singleShot(500, self.start_voice_prompt)
            self.voice_prompt_executed = True

    def start_voice_prompt(self):
        engine = pyttsx3.init()
        text_to_speak = f"{self.map_text2.text()} {self.map_text3.text()}"
        engine.say(text_to_speak)
        engine.runAndWait()

    def return_to_home(self):
        QMetaObject.invokeMethod(self.parent(), "setCurrentIndex", Qt.QueuedConnection, Q_ARG(int, 0))
        self.voice_prompt_executed = False

class IdentificationWidget(QWidget, IdentificationUi):
    def __init__(self, parent=None):
        super(IdentificationWidget, self).__init__(parent)
        self.setupUi(self)
        # camera_label 초기화 및 설정
        self.camera_label = QtWidgets.QLabel(self)  # QLabel 생성
        self.camera_label.setGeometry(50, 50, 640, 480)  # 적절한 위치와 크기로 설정
        self.camera_label.setObjectName("camera_label")  # 객체 이름 설정
        self.thread = None  # 스레드를 초기화하지 않고 시작 시점에 생성
        

    def showEvent(self, event):
        super(IdentificationWidget, self).showEvent(event)
        if self.thread is None:
            self.thread = CameraThread()
            self.thread.change_pixmap_signal.connect(self.update_image)
            self.thread.recognition_complete.connect(self.handle_recognition_result)
            self.thread.start()

    def hideEvent(self, event):
        super(IdentificationWidget, self).hideEvent(event)
        if self.thread is not None:
            self.thread.stop()
            self.thread.wait()
            self.thread = None

    def update_image(self, image):
        self.camera_label.setPixmap(QPixmap.fromImage(image))

    @pyqtSlot(str)
    def handle_recognition_result(self, message):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setText(message)
        msg_box.setWindowTitle("신원 확인 결과")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec_()

        if "신원이 확인되었습니다" in message:
            self.parentWidget().setCurrentIndex(8)

    def closeEvent(self, event):
        if self.thread is not None:
            self.thread.stop()
            self.thread.wait()
        event.accept()


class IdentificationCompleteWidget(QWidget, IdentificationCompleteUi):
    def __init__(self, parent=None):
        super(IdentificationCompleteWidget, self).__init__(parent)
        self.setupUi(self)

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Patient System")
        self.setGeometry(100, 100, 800, 480)

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.home_widget = HomeWidget(self)
        self.webcam_widget = WebcamWidget(self)
        self.record_widget = RecordWidget(self)
        self.pharmacy_map_widget = PharmacyMapWidget(self)
        self.imaging_map_widget = ImagingMapWidget(self)
        self.injection_map_widget = InjectionMapWidget(self)
        self.landmark_complete_widget = landmarkCompleteWidget(self)
        self.identification_widget = IdentificationWidget(self)  # IdentificationWidget 추가
        self.identification_complete_widget = IdentificationCompleteWidget(self)  # IdentificationCompleteWidget 추가

        self.stack.addWidget(self.home_widget) #0
        self.stack.addWidget(self.webcam_widget) #1
        self.stack.addWidget(self.record_widget) #2
        self.stack.addWidget(self.pharmacy_map_widget) #3
        self.stack.addWidget(self.imaging_map_widget) #4
        self.stack.addWidget(self.injection_map_widget) #5
        self.stack.addWidget(self.landmark_complete_widget) #6
        self.stack.addWidget(self.identification_widget)  #7
        self.stack.addWidget(self.identification_complete_widget) #8

        self.setStyleSheet("QMainWindow { background-color: #F0F0F0; }")

        self.stack.setCurrentIndex(0)

    @pyqtSlot()
    def show_identification_complete_widget(self):
        self.stack.setCurrentIndex(8)  # IdentificationCompleteWidget으로 전환


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
