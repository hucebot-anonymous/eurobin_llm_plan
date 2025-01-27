#!/usr/bin/env python3
import time
import queue
import rospy
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wavfile
import json
from enum import Enum

from faster_whisper import WhisperModel
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse
from llama_cpp import Llama, LlamaGrammar

# Constants
MICROPHONE_SR = 48000
TARGET_SR = 16000
NODE_NAME = "whisper_llm"
LOG_LEVEL = Enum("LOG_LEVEL", ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"])


class WhisperLLMService:
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        rospy.loginfo("Whisper service is starting!")

        # ROS service and publishers
        self.service = rospy.Service("whisper_llm", Trigger, self.handle_asr_request)
        rospy.Subscriber("/streamdeck/microphone", Bool, self.speaker_callback)
        self.llm_is_thinking = rospy.Publisher("/llm_is_thinking", Bool, queue_size=1)
        self.stt_pub = rospy.Publisher("/speech_to_text", String, queue_size=1)

        # Variables
        self.comp_q = queue.Queue()
        self.sentence_buffer = ""
        self.void_count = 0
        self.button_state = False
        self.recording = []

        # Paths and model configuration
        self.model_path = "./models"
        self.device = "cuda"
        self.whisper = WhisperModel(
            "large-v3", device=self.device, compute_type="float", download_root=self.model_path
        )
        self.llm = Llama(
            model_path="models/Meta-Llama-3.1-8B-Instruct-Q8_0.gguf",
            n_gpu_layers=-1,
            seed=0,
            n_ctx=4096,
            verbose=False,
        )

        # System prompt
        self.whisper_system_prompt = (
            "You will transcribe sentences about instructions. "
            "The instructions are either of the form 'pick object at location in kitchen and place at location in kitchen', "
            "or 'pick object at location in kitchen and give at person in kitchen'. "
            "The available kitchens are DLR, KIT, and INRIA. "
            "The available locations are cabinet, table, and drawer."
        )

        rospy.loginfo("Models loaded successfully!")

    def speaker_callback(self, msg):
        """Callback to handle microphone button state."""
        self.button_state = msg.data
        rospy.loginfo(f"Microphone state updated: {self.button_state}")

    def asr_callback(self, indata, frames, time, status):
        """Callback for audio input stream."""
        self.recording.append(indata.copy())

    def handle_asr_request(self, req):
        """Handles ASR requests via the ROS service."""
        rospy.loginfo("Waiting for microphone activation...")
        while not self.button_state:
            time.sleep(0.1)

        rospy.loginfo("Recording started.")
        with sd.InputStream(
            device="Wireless GO II RX",
            dtype=np.float32,
            channels=1,
            samplerate=MICROPHONE_SR,
            blocksize=int(0.001 * MICROPHONE_SR),
            callback=self.asr_callback,
        ):
            while self.button_state:
                time.sleep(0.1)

        # Concatenate recorded audio and save it to a WAV file
        recording = np.concatenate(self.recording, axis=0)
        wavfile.write("recording.wav", MICROPHONE_SR, recording)
        rospy.loginfo("Recording saved to 'recording.wav'.")

        # Notify that LLM is processing
        self.llm_is_thinking.publish(True)

        # Transcribe using Whisper
        segments, _ = self.whisper.transcribe(
            "recording.wav", beam_size=5, language="en", initial_prompt=self.whisper_system_prompt
        )
        user_prompt = " ".join(segment.text for segment in segments)
        rospy.loginfo(f"Transcribed text: {user_prompt}")

        # Generate response using LLM
        grammar_path = "/catkin_ws/src/llm_whisper_pkg/grammar/pattern_detector.gbnf"
        grammar = LlamaGrammar.from_file(grammar_path)
        self.generate_prompt(user_prompt)
        response = self.llm.create_chat_completion(
            messages=self.full_prompt, grammar=grammar, temperature=0
        )["choices"][0]["message"]["content"]

        rospy.loginfo(f"Generated response: {response}")
        response_json = json.loads(response)

        # Publish response to the speech-to-text topic
        chain_of_thought = response_json.get("chain_of_thought", "")
        self.stt_pub.publish(chain_of_thought)
        rospy.loginfo(f"Published chain of thought: {chain_of_thought}")

        # Stop LLM thinking indicator
        self.llm_is_thinking.publish(False)

        # Return response to the service
        resp = TriggerResponse()
        resp.success = True
        resp.message = response
        return resp

    def generate_prompt(self, user_prompt):
        """Generates a prompt for the LLM from system and example prompts."""
        system_prompt_path = "/catkin_ws/src/llm_whisper_pkg/prompts/system_prompt.txt"
        examples_path = "/catkin_ws/src/llm_whisper_pkg/prompts/examples.json"

        with open(system_prompt_path, "r") as file:
            system_prompt = file.read()
        with open(examples_path, "r") as file:
            examples = json.load(file)

        self.full_prompt = [{"role": "system", "content": system_prompt}] + examples
        self.full_prompt.append({"role": "user", "content": user_prompt})


if __name__ == "__main__":
    try:
        service = WhisperLLMService()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
