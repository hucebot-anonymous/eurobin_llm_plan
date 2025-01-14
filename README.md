# Speech and Language Processing System for Eurobin Coopetition

This repository implements an advanced **ROS-based speech and language processing framework** designed to support the Eurobin Coopetition. The system integrates **speech-to-text**, **large language models (LLMs)**, and **text-to-speech** functionalities across two interconnected modules.

---

## System Overview

### 1. **ASR + LLM Module (`asr_llm`)**
- **Purpose**: Captures audio from a microphone, transcribes it to text, and generates a structured plan that satisfies the Eurobin Coopetition setup.
- **Key Outputs**:
  - **Generated Plan**: A structured plan derived using advanced language models and custom grammars.
    - **Chain of Thought**: A step-by-step reasoning process (part of the generated plan) published for downstream use.
- **Technology Stack**:
  - [**faster-whisper**](https://github.com/SYSTRAN/faster-whisper)
  - [**Llama-cpp-python**](https://github.com/abetlen/llama-cpp-python)

### 2. **Speech Generation Module (`speech_gen`)**
- **Purpose**: Takes the **Chain of Thought** output from the `asr_llm` module and generates high-quality synthetic speech for human interaction.
- **Technology Stack**:
    - [**coqui-tts**](https://github.com/coqui-ai/TTS)

---

## Note on Maintenance

This repository is currently under maintenance, and certain features may be incomplete or subject to change. More detailed documentation, usage instructions, and examples will be added in future updates.