# Base image with NVIDIA runtime for CUDA 12.4.1 on Ubuntu 20.04
FROM nvidia/cuda:12.4.1-runtime-ubuntu20.04

# Argument for specifying CUDNN architecture
ARG CUDNN_ARCH="linux-x86_64"

# Environment variables
ENV TZ="Europe/Paris"
ENV SHELL="/bin/bash"
ENV TERM="xterm"
ENV PATH="/opt/ros/noetic/bin:${PATH}"

# Set working directory
WORKDIR /libs

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    software-properties-common \
    pulseaudio-utils \
    libsndfile1 \
    espeak-ng \
    libportaudio2 && \
    rm -rf /var/lib/apt/lists/*

# Install CUDA toolkit
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y cuda-toolkit-12-4 && \
    rm -rf /var/lib/apt/lists/*

# Install CUDNN
RUN wget https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/${CUDNN_ARCH}/cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive.tar.xz && \
    tar -xvf cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive.tar.xz && \
    cp -P cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive/include/* /usr/local/cuda/include/ && \
    cp -P cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive/lib/* /usr/local/cuda/lib64/ && \
    rm -rf cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive*

# Install Python 3.11 and dependencies
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.11 python3.11-distutils python-is-python3 python3-pip python3.11-dev && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip install --no-cache-dir \
    sounddevice \
    pydub \
    TTS \
    protobuf --upgrade

# Install ROS1
WORKDIR /temp
COPY .ci/install_ros1.sh /temp/install_ros1.sh
RUN chmod +x /temp/install_ros1.sh && \
    /temp/install_ros1.sh && \
    rm -rf /temp

# Configure ROS environment
RUN echo "export ROS_MASTER_URI=http://192.168.50.162:11311" >> ~/.bashrc && \
    echo "export ROS_HOSTNAME=192.168.50.221" >> ~/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash"]
