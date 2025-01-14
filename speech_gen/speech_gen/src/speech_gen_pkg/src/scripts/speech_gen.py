#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import soundfile as sf
import torch
from TTS.api import TTS

class SpeechToTextToSpeech:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('speech_to_text_to_speech', anonymous=True)

        # Load the TTS model
        self.tts = TTS("tts_models/en/ljspeech/vits", gpu=torch.cuda.is_available())
        rospy.loginfo("Loaded TTS model: tts_models/en/ljspeech/vits")

        # Subscribe to /speech_to_text topic
        self.subscriber = rospy.Subscriber("/speech_to_text", String, self.callback)

        # Audio publisher
        self.audio_pub = rospy.Publisher('/audio/audio', AudioData, queue_size=10)

    def callback(self, msg):
        """
        Callback function to handle received text and convert it to speech.
        """
        text = msg.data
        rospy.loginfo(f"Received text: {text}")

        output_path = "output.wav"
        try:
            # Convert text to speech and save to a file
            self.tts.tts_to_file(text=text, speaker_wav=None, file_path=output_path)
            rospy.loginfo(f"Generated audio saved to {output_path}")

            # Load the generated audio file
            data, samplerate = sf.read(output_path, dtype='int16')
            rospy.loginfo(f"Audio sample rate: {samplerate}")

            # Convert audio data to bytes and publish
            audio_msg = AudioData()
            audio_msg.data = data.tobytes()
            self.audio_pub.publish(audio_msg)

            rospy.loginfo("Published audio data to /audio/audio")
        except Exception as e:
            rospy.logerr(f"Error processing text-to-speech: {e}")

if __name__ == '__main__':
    try:
        SpeechToTextToSpeech()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down SpeechToTextToSpeech node.")
