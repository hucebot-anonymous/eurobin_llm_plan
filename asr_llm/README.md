# Whisper LLM Service

The **Whisper LLM Service** is a ROS-based application that integrates speech-to-text capabilities using **faster-whisper** (a highly optimized version of Whisper) and advanced language modeling using Llama. It is designed to transcribe spoken instructions into structured outputs and generate intelligent responses based on custom grammar patterns.

---

## Directory Structure

### Root Level
- `.ci/`: Continuous integration setup, including Dockerfile and ROS installation script.
- `launch.sh`: Script to initialize the application.
- `models/`: Contains pre-trained models for **faster-whisper** and Llama.
- `README.md`: Documentation (you are here).
- `src/`: Source code for ROS packages.

### `src` Directory
- **`eurobin_custom_services/`**:
  - Contains a custom ROS service (`WhisperTranscribe.srv`) and package configurations.
- **`llm_whisper_pkg/`**:
  - Main package for integrating **faster-whisper** and Llama models.
  - Subdirectories:
    - `grammar/`: Grammar files defining custom LLM behavior.
    - `models/`: you should download the .gguf LLMs and place them here, also the whisper models will be cached here.
    - `prompts/`: Contains prompt configurations for system and example-based prompting.
    - `src/scripts/`: Main Python script (`llm.py`) for processing requests.


## Usage

### Modify the Dockerfile
Before building the Docker image, you need to update the following lines in the `Dockerfile` to configure the ROS environment:

```Dockerfile
# Configure ROS environment
RUN echo 'export ROS_MASTER_URI=' >> ~/.bashrc && \
    echo 'export ROS_IP=' >> ~/.bashrc 
```

Set `ROS_MASTER_URI` and `ROS_IP` to the appropriate values for your network setup.

---

### Build the Docker Image
To build the Docker image, navigate to the root directory of the project and run:

```bash
docker build -t llm_whisper -f .ci/Dockerfile .
```

---

### Download LLM Models
Before launching the container, download the required LLM models and place them in the `models/` directory. The codebase expects the following model by default:  
[**Meta-Llama-3.1-8B-Instruct-Q8_0.gguf**](https://huggingface.co/bartowski/Meta-Llama-3.1-8B-Instruct-GGUF/blob/main/Meta-Llama-3.1-8B-Instruct-Q8_0.gguf).  

You can use a different model if preferred. If so, update the `llm_whisper_pkg/src/scripts/llm.py` file to reference your selected model.

---

### Launch the Container
To start the container, execute:

```bash
bash launch.sh
```

---

### Start the `whisper_llm` Service
Inside the container, run the following commands to set up and start the `whisper_llm` service:

```bash
catkin_make
source devel/setup.bash
rosrun llm_whisper_pkg llm.py
```

---

### Push-to-Talk and Microphone Configuration
- The audio is transcribed only when the `/streamdeck/microphone` topic is set to `true` while the user is speaking. ([We use the Stream Deck as a push-to-talk button.](https://github.com/hucebot/stream_deck_controller))  
- By default, the code assumes you are using the **Wireless GO II** microphone. If using a different microphone, update the `llm_whisper_pkg/src/scripts/llm.py` file to specify the correct microphone.

---

### Service Outputs
Once audio is transcribed by Whisper and processed by the LLM:  
1. The JSON response is returned to the service caller.  
2. The "chain of thought" field is published to the `/speech_to_text` topic (for use by the `speech_gen` node).

During this process, the following topics are updated to reflect the system's state for onboard display purposes:  
- `llm_is_thinking`



## Features

- **Speech-to-Text (ASR)**:
  - Uses **faster-whisper** for optimized transcription with GPU acceleration.
  - Supports real-time audio capture and user-defined prompts.

- **Language Modeling**:
  - Leverages Llama-cpp for structured text generation based on input prompts and predefined grammars.

- **Custom Grammar Support**:
  - Incorporates grammar-based constraints using GBNF files for robust language parsing.

- **ROS Integration**:
  - Exposes services (`whisper_llm`) and topics (`/llm_is_thinking`, `/speech_to_text`) for interaction within a ROS ecosystem.
