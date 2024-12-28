#!/usr/bin/env python3
import time
import queue
import threading
import rospy
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wavfile
import json
import re

from faster_whisper import WhisperModel
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse
from enum import Enum

from llama_cpp import Llama
from llama_cpp import LlamaGrammar

LOG_LEVEL = Enum("LOG_LEVEL", ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"])


MICROPHONE_SR = 48000
TARGET_SR = 16000
NODE_NAME = "whisper_llm"


class WhisperLLMService:
    def __init__(self):
        rospy.init_node("demo_llm_service", anonymous=True)

        rospy.loginfo("Whisper service is starting !")

        self.service = rospy.Service(
            "demo_llm", Trigger, self.handle_asr_request)
        self.comp_q = queue.Queue()
        self.sentence_buffer = ""
        self.void_count = 0
        self.button_state = False
        self.model_path = "./models"
        self.model = "large-v3"
        self.device = "cuda"

        self.whisper = WhisperModel(
            self.model,
            device=self.device,
            compute_type="float",
            download_root=self.model_path,
        )

        self.llm = Llama(
            model_path="models/Meta-Llama-3.1-8B-Instruct-Q8_0.gguf",
            n_gpu_layers=-1,
            seed=0,
            n_ctx=2048*2,
            verbose=False
        )

        # self.MIC_DEVICE = "Wireless GO II RX"
        self.SAMPLE_RATE = 48000

        self.system_prompt="You will transcribe sentences about instructions. \
        The instructions are either of the form pick object at location in kitchen and place at location in kitchen.\
        or pick object at location in kitchen and gice at person in kitchen. \
        The available kitchens are DLR, KIT and INRIA.\
        The available locations are cabinet, table, drawer."

        # listener to the '/streamdeck/speaker' --> Bool
        rospy.Subscriber("/streamdeck/microphone", Bool, self.speaker_callback)
        self.llm_is_thinking = rospy.Publisher("/llm_is_thinking", Bool, queue_size=1)
        self.stt_pub = rospy.Publisher("/speech_to_text", String, queue_size=1)

        # self.stream.start()

        self.history = []

        rospy.loginfo("Models loaded successfuly!")

    def speaker_callback(self, msg):
        # if the topic is True set the self.button_state to True
        print(msg.data)
        if msg.data:
            self.button_state = True
        else:
            self.button_state = False

    def asr_callback(self, indata, frames, time, status):
        self.recording.append(indata.copy())

    def handle_asr_request(self, req):
        self.recording = []
        # Wait for the button to be pressed
        while not self.button_state:
            rospy.loginfo("Waiting for the button to be pressed")
            time.sleep(0.1)

        # Button is pressed, start the recording stream
        with sd.InputStream(
            device="Wireless GO II RX",
            dtype=np.float32,
            channels=1,
            samplerate=self.SAMPLE_RATE,
            blocksize=int(0.001 * self.SAMPLE_RATE),  # corrected reference to MICROPHONE_SR
            callback=self.asr_callback,
        ):
            while self.button_state:
                time.sleep(0.1)

        # Concatenate recorded segments
        recording = np.concatenate(self.recording, axis=0)

        # TO-DO: Find a way to avoid this unnecesary writing operation
        # Save recording to a WAV file
        wavfile.write("recording.wav", self.SAMPLE_RATE, recording)
        rospy.loginfo("Recording saved to recording.wav")

        # publish to the /llm_is_thinking topic
        self.llm_is_thinking.publish(True)
        # Per`form tran`scription using the ASR model
        segments, info = self.whisper.transcribe(
            "recording.wav",
            beam_size=5,
            language="en",
            initial_prompt=self.system_prompt
        )

        # Log each transcribed segment
        user_prompt = ''
        for segment in segments:
            rospy.loginfo(f"[{segment.start:.2f}s -> {segment.end:.2f}s] {segment.text}")
            user_prompt += segment.text + " "
        # Concatenate all segment texts into a single sentence
        # user_prompt = ''.join(segment.text for segment in segments)

        rospy.loginfo("Generating response for the prompt: %s", user_prompt)
        grammar = LlamaGrammar.from_file("/catkin_ws/src/llm_whisper_pkg/grammar/demo.gbnf")


        self.generate_prompt(user_prompt)

        response = self.llm.create_chat_completion(messages = self.full_prompt, grammar=grammar, temperature=1)['choices'][0]['message']['content']

        rospy.loginfo(response)
        response_json = json.loads(response)

        # publish llm_is_thinking to False

        # get the chain_of_thought field and publish it to the /speech_to_text topic
        chain_of_thought = response_json['chain_of_thought']
        # pub = rospy.Publisher("/speech_to_text", String, queue_size=1)
        self.stt_pub.publish(chain_of_thought)
        # rospy.loginfo("LLM Generated plan:\n", response_json)

        # response to ascii
        # response = response.encode('ascii')
        # Return the final transcribed sentence

        resp = TriggerResponse()
        resp.success = True
        resp.message = response

        self.history.append({"role": "user", "content": user_prompt})
        self.history.append({"role": "system", "content": response})


        # if the history is greater than 10, remove the first two elements
        if len(self.history) > 10:
            self.history = self.history[2:]

        return resp

    def generate_prompt(self, user_prompt):
        with open("/catkin_ws/src/llm_whisper_pkg/prompts/demo_system_prompt.txt", "r") as file:
            system_prompt = file.read()

        # load json
        with open("/catkin_ws/src/llm_whisper_pkg/prompts/demo_examples.json", "r") as file:
            examples = json.load(file)

        print(examples)

        messages = [{"role": "system", "content": system_prompt}] + examples

        # messages += self.history

        self.full_prompt = messages + [{"role": "user", "content": user_prompt}]





if __name__ == "__main__":
    try:
        ws = WhisperLLMService()
        rospy.spin()  # Keep the service alive
    except rospy.ROSInterruptException:
        pass
