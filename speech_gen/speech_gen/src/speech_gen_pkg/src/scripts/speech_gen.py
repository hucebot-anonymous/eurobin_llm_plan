#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import soundfile as sf
import torch
from TTS.api import TTS  # Assuming this is a TTS API you're using
import pydub

class SpeechToTextToSpeech:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('speech_to_text_to_speech', anonymous=True)


        # ['tts_models/multilingual/multi-dataset/your_tts', 'tts_models/bg/cv/vits', 'tts_models/cs/cv/vits', 'tts_models/da/cv/vits', 'tts_models/et/cv/vits', 'tts_models/ga/cv/vits', 'tts_models/en/ek1/tacotron2', 'tts_models/en/ljspeech/tacotron2-DDC', 'tts_models/en/ljspeech/tacotron2-DDC_ph', 'tts_models/en/ljspeech/glow-tts', 'tts_models/en/ljspeech/speedy-speech', 'tts_models/en/ljspeech/tacotron2-DCA', 'tts_models/en/ljspeech/vits', 'tts_models/en/ljspeech/vits--neon', 'tts_models/en/ljspeech/fast_pitch', 'tts_models/en/ljspeech/overflow', 'tts_models/en/ljspeech/neural_hmm', 'tts_models/en/vctk/vits', 'tts_models/en/vctk/fast_pitch', 'tts_models/en/sam/tacotron-DDC', 'tts_models/en/blizzard2013/capacitron-t2-c50', 'tts_models/en/blizzard2013/capacitron-t2-c150_v2', 'tts_models/en/multi-dataset/tortoise-v2', 'tts_models/en/jenny/jenny']


        # ['tts_models/multilingual/multi-dataset/your_tts', 'tts_models/bg/cv/vits', 'tts_models/cs/cv/vits', 'tts_models/da/cv/vits', 'tts_models/et/cv/vits', 'tts_models/ga/cv/vits', 'tts_models/en/ek1/tacotron2', 'tts_models/en/ljspeech/tacotron2-DDC', 'tts_models/en/ljspeech/tacotron2-DDC_ph', 'tts_models/en/ljspeech/glow-tts', 'tts_models/en/ljspeech/speedy-speech', 'tts_models/en/ljspeech/tacotron2-DCA', 'tts_models/en/ljspeech/vits', 'tts_models/en/ljspeech/vits--neon', 'tts_models/en/ljspeech/fast_pitch', 'tts_models/en/ljspeech/overflow', 'tts_models/en/ljspeech/neural_hmm', 'tts_models/en/vctk/vits', 'tts_models/en/vctk/fast_pitch', 'tts_models/en/sam/tacotron-DDC', 'tts_models/en/blizzard2013/capacitron-t2-c50', 'tts_models/en/blizzard2013/capacitron-t2-c150_v2', 'tts_models/en/multi-dataset/tortoise-v2', 'tts_models/en/jenny/jenny', 'tts_models/es/mai/tacotron2-DDC', 'tts_models/es/css10/vits', 'tts_models/fr/mai/tacotron2-DDC', 'tts_models/fr/css10/vits', 'tts_models/uk/mai/glow-tts', 'tts_models/uk/mai/vits', 'tts_models/zh-CN/baker/tacotron2-DDC-GST', 'tts_models/nl/mai/tacotron2-DDC', 'tts_models/nl/css10/vits', 'tts_models/de/thorsten/tacotron2-DCA', 'tts_models/de/thorsten/vits', 'tts_models/de/thorsten/tacotron2-DDC', 'tts_models/de/css10/vits-neon', 'tts_models/ja/kokoro/tacotron2-DDC', 'tts_models/tr/common-voice/glow-tts', 'tts_models/it/mai_female/glow-tts', 'tts_models/it/mai_female/vits', 'tts_models/it/mai_male/glow-tts', 'tts_models/it/mai_male/vits', 'tts_models/ewe/openbible/vits', 'tts_models/hau/openbible/vits', 'tts_models/lin/openbible/vits', 'tts_models/tw_akuapem/openbible/vits', 'tts_models/tw_asante/openbible/vits', 'tts_models/yor/openbible/vits', 'tts_models/hu/css10/vits', 'tts_models/el/cv/vits', 'tts_models/fi/css10/vits', 'tts_models/hr/cv/vits', 'tts_models/lt/cv/vits', 'tts_models/lv/cv/vits', 'tts_models/mt/cv/vits', 'tts_models/pl/mai_female/vits', 'tts_models/pt/cv/vits', 'tts_models/ro/cv/vits', 'tts_models/sk/cv/vits', 'tts_models/sl/cv/vits', 'tts_models/sv/cv/vits', 'tts_models/ca/custom/vits', 'tts_models/fa/custom/glow-tts', 'tts_models/bn/custom/vits-male', 'tts_models/bn/custom/vits-female']

        # Load TTS model
        self.tts = TTS("tts_models/en/ljspeech/vits",
                       gpu=torch.cuda.is_available())


        # self.tts = TTS("tts_models/fr/mai/tacotron2-DDC",
        #                   gpu=torch.cuda.is_available())
        rospy.loginfo("Loaded TTS model")

        # Subscribe to /speech_to_text topic
        self.subscriber = rospy.Subscriber(
            "/speech_to_text", String, self.callback)

        # Audio publisher
        self.audio_pub = rospy.Publisher(
            '/audio/audio', AudioData, queue_size=10)

    def callback(self, msg):
        # Extract text from the message
        text = msg.data
        rospy.loginfo(f"Received text: {text}")

        # Convert text to speech and save to file
        output_path = "output.wav"
        self.tts.tts_to_file(text=text, speaker_wav=None,
                             file_path=output_path)


        # Load the entire audio file at once
        data, samplerate = sf.read(output_path, dtype='int16')

        print("Sample rate: ", samplerate)


        # Convert the audio data to bytes for the AudioData message
        audio_msg = AudioData()
        audio_msg.data = data.tobytes()  # Convert entire array to bytes


        # Publish the full audio data as a single message
        self.audio_pub.publish(audio_msg)

        rospy.loginfo("Published complete audio data as a single message")


if __name__ == '__main__':
    try:
        SpeechToTextToSpeech()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
