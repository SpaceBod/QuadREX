import os
from dotenv import load_dotenv
from ibm_watson import AssistantV2
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from wakeWordDetection import PorcupineWakeWordDetector, GoogleSpeechToText, ElevenLabsTTS, hash_text
import requests

class WatsonAssistant:
    def __init__(self):
        load_dotenv()

        # Initialise the Watson Assistant
        self.api_key = os.getenv("WATSON_API_KEY")
        self.service_url = os.getenv("WATSON_SERVICE_URL")
        self.assistant_id = os.getenv("WATSON_ASSISTANT_ID")
        self.thingspeak_api_key = os.getenv("THINGSPEAK_KEY")
        self.thingspeak_channel_id = os.getenv("THINGSPEAK_ID")

        if not self.api_key or not self.service_url or not self.assistant_id:
            raise ValueError(
                "Missing one or more environment variables for Watson Assistant"
            )

        self.authenticator = IAMAuthenticator(self.api_key)
        self.assistant = AssistantV2(
            version="2021-06-14", authenticator=self.authenticator
        )
        self.assistant.set_service_url(self.service_url)

        # Initialise ElevenLabs TTS
        self.tts = ElevenLabsTTS()

    def check_continue_previous_topic(self, text):
        return (
            text.strip()
            .lower()
            .startswith("do you want to continue with the previous topic")
        )

    def publish_to_thingspeak(self, details):
        url = f"https://api.thingspeak.com/update.json"
        payload = {
            "api_key": self.thingspeak_api_key,
            "field1": details[0],
            "field2": details[1],
            "field3": details[2],
            "field4": details[3],
            "field5": details[4],
            "field6": details[5],
        }
        response = requests.post(url, data=payload)
        if response.status_code == 200:
            print("Data successfully published to ThingSpeak.")
        else:
            print("Failed to publish data to ThingSpeak:", response.text)

    def checkData(self, response, session_id):
        required_keys = [
            "Client_Age",
            "Client_Name",
            "Symptom_Details",
            "Symptom_Intensity",
            "Symptom_Time",
            "Symptom_Type",
        ]

        context = response.get("context", {})
        skills = context.get("skills", {})
        actions_skill = skills.get("actions skill", {})
        skill_variables = actions_skill.get("skill_variables", {})

        # Extract and check if all required values are present
        data = {key: skill_variables.get(key, None) for key in required_keys}

        if all(value is not None for value in data.values()):

            sessionDetails = [
                {data["Symptom_Time"]["value"]},
                {data["Client_Name"]},
                {data["Client_Age"]},
                {data["Symptom_Type"]},
                {data["Symptom_Details"]},
                {data["Symptom_Intensity"]},
            ]

            print(sessionDetails)
            self.publish_to_thingspeak(sessionDetails)

            # Close the current session and create a new one
            self.assistant.delete_session(
                assistant_id=self.assistant_id, session_id=session_id
            )
            new_session_id = self.assistant.create_session(
                assistant_id=self.assistant_id
            ).get_result()["session_id"]
            print("Session reset. New session ID:", new_session_id)
            return new_session_id

        return session_id

    def interact(self, spoken_text):
        session_id = self.assistant.create_session(
            assistant_id=self.assistant_id
        ).get_result()["session_id"]

        try:
            while True:
                if spoken_text is None:
                    retry_message = "I didn't catch that. Could you please repeat?"
                    hashed_filename = hash_text(retry_message)
                    output_file = os.path.join(
                        self.tts.output_folder, f"{hashed_filename}.mp3"
                    )

                    if not os.path.exists(output_file):
                        self.tts.generate_audio(retry_message, output_file)
                    self.tts.play_audio(output_file)

                    stt = GoogleSpeechToText()
                    spoken_text = stt.listen_and_transcribe()
                    continue

                if spoken_text.lower() == "exit":
                    print("Ending conversation.")
                    break

                response = self.assistant.message(
                    assistant_id=self.assistant_id,
                    session_id=session_id,
                    input={
                        "message_type": "text",
                        "text": spoken_text,
                        "options": {"return_context": True},
                    },
                ).get_result()

                # Extract data from conversation and check it
                session_id = self.checkData(response, session_id)

                output = response["output"]["generic"]
                for message in output:
                    if message["response_type"] == "text":
                        print(f"Assistant: {message['text']}")

                        # Check if the assistant asks to continue with the previous topic
                        if self.check_continue_previous_topic(message["text"]):
                            response = self.assistant.message(
                                assistant_id=self.assistant_id,
                                session_id=session_id,
                                input={"message_type": "text", "text": "no"},
                            ).get_result()
                            break  # Restart the loop to handle the new response

                        # Generate and play the audio response
                        hashed_filename = hash_text(message["text"])
                        output_file = os.path.join(
                            self.tts.output_folder, f"{hashed_filename}.mp3"
                        )

                        if not os.path.exists(output_file):
                            self.tts.generate_audio(message["text"], output_file)
                        self.tts.play_audio(output_file)
                    elif message["response_type"] == "option":
                        if "title" in message:
                            print(f"Assistant: {message['title']}")
                        for option in message.get("options", []):
                            print(f"- {option['label']}")
                # Check for actions or additional steps if necessary
                if "actions" in response["output"]:
                    for action in response["output"]["actions"]:
                        print(f"Action: {action['name']} - {action['parameters']}")

                # Listen for next user input
                stt = GoogleSpeechToText()
                spoken_text = stt.listen_and_transcribe()

        finally:
            self.assistant.delete_session(
                assistant_id=self.assistant_id, session_id=session_id
            )

if __name__ == "__main__":
    detector = PorcupineWakeWordDetector(
        keyword_paths=["./assets/models/Help-me_en_raspberry-pi_v3_0_0.ppn", "./assets/models/Hey-Rex_en_raspberry-pi_v3_0_0.ppn"]
    )

    try:
        while True:
            detector.listen_for_wake_word()

            stt = GoogleSpeechToText()
            spoken_text = stt.listen_and_transcribe()

            if spoken_text:
                assistant = WatsonAssistant()
                assistant.interact(spoken_text)

    except KeyboardInterrupt:
        print(Fore.RED + "\n[EXIT] Program terminated by user.")
        detector.cleanup()
