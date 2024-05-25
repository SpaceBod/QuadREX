import os
import hashlib
import pygame
import requests
from dotenv import load_dotenv
from colorama import Fore, init

init(autoreset=True)


class ElevenLabsTTS:
    CHUNK_SIZE = 1024

    def __init__(self):
        load_dotenv()
        self.api_key = os.getenv("XI_API_KEY")
        self.voice_id = os.getenv("XI_VOICE_ID")
        self.output_folder = "assets/audio"
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
