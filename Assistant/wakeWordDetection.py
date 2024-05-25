import os
from colorama import Fore, Style, init
import hashlib
import numpy as np
import pvporcupine
import pyaudio
import pygame
from dotenv import load_dotenv
import speech_recognition as sr
import requests

init(autoreset=True)


class PorcupineWakeWordDetector:
    def __init__(self, keyword_paths, sensitivity=0.7):
        load_dotenv()
        self.access_key = os.getenv("PORCUPINE_KEY")
        self.keyword_paths = keyword_paths
        self.sensitivity = [sensitivity] * len(keyword_paths)

        self.porcupine = pvporcupine.create(
            access_key=self.access_key,
            keyword_paths=self.keyword_paths,
            sensitivities=self.sensitivity,
        )
        self.pa = pyaudio.PyAudio()
        self.audio_stream = self.pa.open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length,
        )

    def listen_for_wake_word(self):
        print(Fore.YELLOW + "[IDLE] Listening for wake word...")
        try:
            while True:
                pcm = self.audio_stream.read(
                    self.porcupine.frame_length, exception_on_overflow=False
                )
                pcm = np.frombuffer(pcm, dtype=np.int16)

                keyword_index = self.porcupine.process(pcm)
                if keyword_index >= 0:
                    self.play_sound("assets/sfx/beep.mp3")
                    break
        except KeyboardInterrupt:
            self.cleanup()
            raise

    def play_sound(self, file_path):
        pygame.mixer.init()
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

    def cleanup(self):
        if self.porcupine is not None:
            self.porcupine.delete()
        if self.audio_stream is not None:
            self.audio_stream.close()
        if self.pa is not None:
            self.pa.terminate()


class GoogleSpeechToText:
    def __init__(self):
        self.recognizer = sr.Recognizer()

    def listen_and_transcribe(self):
        with sr.Microphone() as source:
            print(Fore.GREEN + "[READY] Listening for speech...")
            audio = self.recognizer.listen(source)
            self.play_sound("assets/sfx/beep.mp3")
            try:
                text = self.recognizer.recognize_google(audio)
                print(Fore.BLUE + "[INPUT] " + text)
                return text
            except sr.UnknownValueError:
                print(Fore.RED + "Google Speech Recognition could not understand audio")
            except sr.RequestError as e:
                print(
                    Fore.RED
                    + f"Could not request results from Google Speech Recognition service; {e}"
                )

        return None

    def play_sound(self, file_path):
        pygame.mixer.init()
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)


class ElevenLabsTTS:
    CHUNK_SIZE = 1024

    def __init__(self):
        load_dotenv()
        self.api_key = os.getenv("XI_API_KEY")
        self.voice_id = "wfvZgVT0AcxhJmfhcvxo"
        self.output_folder = "mp3s"
        os.makedirs(self.output_folder, exist_ok=True)

    def generate_audio(self, text, file_path):
        tts_url = f"https://api.elevenlabs.io/v1/text-to-speech/{self.voice_id}/stream"

        headers = {
            "Accept": "application/json",
            "xi-api-key": self.api_key,
            "Content-Type": "application/json",
        }

        data = {
            "text": text,
            "model_id": "eleven_multilingual_v1",
            "voice_settings": {"stability": 0.5, "similarity_boost": 0.8},
        }

        response = requests.post(tts_url, json=data, headers=headers, stream=True)

        with open(file_path, "wb") as f:
            for chunk in response.iter_content(chunk_size=self.CHUNK_SIZE):
                if chunk:
                    f.write(chunk)

        print(Fore.MAGENTA + f"[MP3] Finished writing audio file to {file_path}")

    def play_audio(self, file_path):
        pygame.mixer.init()
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)


def hash_text(text):
    return hashlib.md5(text.encode()).hexdigest()


if __name__ == "__main__":
    keyword_paths = ["./assets/models/windows.ppn", "./assets/models/windows-rex.ppn"]
    detector = PorcupineWakeWordDetector(keyword_paths=keyword_paths)

    try:
        while True:
            detector.listen_for_wake_word()

            stt = GoogleSpeechToText()
            spoken_text = stt.listen_and_transcribe()

            if spoken_text:
                tts = ElevenLabsTTS()
                hashed_filename = hash_text(spoken_text)
                output_file = os.path.join(tts.output_folder, f"{hashed_filename}.mp3")

                if not os.path.exists(output_file):
                    tts.generate_audio(spoken_text, output_file)
                tts.play_audio(output_file)

            print(Fore.YELLOW + "[END] Conversation ended..")
    except KeyboardInterrupt:
        print(Fore.RED + "\n[EXIT] Program terminated by user.")
        detector.cleanup()