import os
from dotenv import load_dotenv
from ibm_watson import AssistantV2
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from text2speech import ElevenLabsTTS, hash_text


class WatsonAssistant:
    def __init__(self):
        load_dotenv()

        # Initialise the Watson Assistant
        self.api_key = os.getenv("WATSON_API_KEY")
        self.service_url = os.getenv("WATSON_SERVICE_URL")
        self.assistant_id = os.getenv("WATSON_ASSISTANT_ID")

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

    def interact(self):
        print("Type 'exit' to end the conversation.")
        session_id = self.assistant.create_session(
            assistant_id=self.assistant_id
        ).get_result()["session_id"]

        try:
            while True:
                user_input = input("You: ")
                if user_input.lower() == "exit":
                    print("Ending conversation.")
                    break

                response = self.assistant.message(
                    assistant_id=self.assistant_id,
                    session_id=session_id,
                    input={"message_type": "text", "text": user_input},
                ).get_result()

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

        finally:
            self.assistant.delete_session(
                assistant_id=self.assistant_id, session_id=session_id
            )


if __name__ == "__main__":
    assistant = WatsonAssistant()
    assistant.interact()
