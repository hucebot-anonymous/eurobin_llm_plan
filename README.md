# Speech and Language Processing System for Eurobin Coopetition

![](assets/overview.png)

This repository implements a **ROS-based speech and language processing framework** designed to support the Eurobin Coopetition. The system integrates **speech-to-text**, **large language models (LLMs)**, and **text-to-speech** functionalities across two interconnected modules.

---

## System Overview

### 1. **ASR + LLM Module ([`asr_llm`](https://github.com/hucebot/eurobin_llm_plan/tree/main/asr_llm))**
- **Purpose**: Captures audio from a microphone, transcribes it to text, and generates a structured plan that satisfies the Eurobin Coopetition setup.
- **Key Outputs**:
  - **Generated Plan**: A structured plan derived using advanced language models and custom grammars.
    - **Chain of Thought**: A step-by-step reasoning process (part of the generated plan) published for downstream use.
- **Technology Stack**:
  - [**faster-whisper**](https://github.com/SYSTRAN/faster-whisper)
  - [**Llama-cpp-python**](https://github.com/abetlen/llama-cpp-python)

### 2. **Speech Generation Module ([`speech_gen`](https://github.com/hucebot/eurobin_llm_plan/tree/main/speech_gen))**
- **Purpose**: Takes the **Chain of Thought** output from the `asr_llm` module and generates high-quality synthetic speech for human interaction.
- **Technology Stack**:
    - [**coqui-tts**](https://github.com/coqui-ai/TTS)

