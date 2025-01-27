FROM nvidia/cuda:12.4.1-runtime-ubuntu20.04

# Arguments and environment variables
ARG DEBIAN_FRONTEND=noninteractive
ARG CUDNN_ARCH="linux-x86_64"
ENV TZ="Europe/Paris"
ENV SHELL=/bin/bash
ENV TERM=xterm
ENV PATH="/opt/ros/noetic/bin:${PATH}"

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Working directory for initial setup
WORKDIR /libs

# Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    software-properties-common && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install CUDA toolkit
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && apt-get install -y cuda-toolkit-12-4 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install cuDNN from tarball
RUN wget https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/${CUDNN_ARCH}/cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive.tar.xz && \
    tar -xvf cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive.tar.xz && \
    cp cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive/include/* /usr/local/cuda/include/ && \
    cp cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive/lib/* /usr/local/cuda/lib64/ && \
    rm -rf cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive*

# Install Python 3.11 and pip dependencies
RUN add-apt-repository ppa:deadsnakes/ppa && apt-get update && \
    apt-get install -y python3.11 python3.11-distutils python-is-python3 python3-pip python3.11-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip install --no-cache-dir \
    faster-whisper \
    sounddevice \
    pydub \
    matplotlib \
    scipy

# Install PortAudio
RUN apt-get update && apt-get install -y --no-install-recommends portaudio19-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install llama-cpp-python with CMake arguments
RUN CMAKE_ARGS="-DGGML_CUDA=on" pip install --no-cache-dir llama-cpp-python

# ROS1 installation setup
WORKDIR /temp
COPY .ci/install_ros1.sh /temp/install_ros1.sh
RUN chmod +x /temp/install_ros1.sh && \
    /temp/install_ros1.sh && \
    cd .. && rm -rf /temp

# ROS workspace setup
WORKDIR /catkin_ws

# Configure ROS environment
RUN echo 'export ROS_MASTER_URI=' >> ~/.bashrc && \
    echo 'export ROS_IP=' >> ~/.bashrc 

# Default to bash
ENTRYPOINT ["/bin/bash"]
