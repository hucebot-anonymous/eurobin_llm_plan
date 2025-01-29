#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String, Bool
from llama_cpp import Llama, LlamaGrammar

# Constants
NODE_NAME = "llm_planner"

class LLMPlanner:
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        rospy.loginfo("LLM Planner node is running!")       
        self.plan_pub = rospy.Publisher("/plan", String, queue_size=1)
        
        # ROS subscriber
        rospy.Subscriber("/user_prompt", String, self.prompt_callback)

        # Load the LLM model
        self.llm = Llama(
            model_path="models/Meta-Llama-3.1-8B-Instruct-Q8_0.gguf",
            n_gpu_layers=-1,
            seed=0,
            n_ctx=4096,
            verbose=False,
        )

        rospy.loginfo("LLM model loaded successfully!")

    def prompt_callback(self, msg):
        """Callback to process incoming prompt strings."""
        user_prompt = msg.data
        rospy.loginfo(f"Received prompt: {user_prompt}")

        # Generate response using the LLM
        grammar_path = "/catkin_ws/src/llm_whisper_pkg/grammar/pattern_detector.gbnf"
        grammar = LlamaGrammar.from_file(grammar_path)
        self.generate_prompt(user_prompt)
        response = self.llm.create_chat_completion(
            messages=self.full_prompt, grammar=grammar, temperature=0
        )["choices"][0]["message"]["content"]
        
        response_json = json.loads(response)
        
        rospy.loginfo(f"Generated response: {json.dumps(response_json, indent=4)}")
        
        # Publish response to the s
        self.plan_pub.publish(response)
        rospy.loginfo(f"Published plan")



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
        llm_planner = LLMPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
