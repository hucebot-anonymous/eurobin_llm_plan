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

---

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
