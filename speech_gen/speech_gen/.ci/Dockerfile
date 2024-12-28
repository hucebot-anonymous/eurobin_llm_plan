FROM nvidia/cuda:12.4.1-runtime-ubuntu20.04


ARG CUDNN_ARCH="linux-x86_64"

ENV TZ "Europe/Paris"

RUN apt-get update \
 && apt-get install --yes pulseaudio-utils


COPY conf/pulse-client.conf /etc/pulse/client.conf

# Set the default shell to bash instead of sh
SHELL ["/bin/bash", "-c"]

ENV SHELL /bin/bash
ENV TERM xterm

WORKDIR /libs


# install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    software-properties-common

# cuda toolkit
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get install -y cuda-toolkit-12-4


# install cudnn8 from tarball
RUN wget https://developer.download.nvidia.com/compute/cudnn/redist/cudnn/${CUDNN_ARCH}/cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive.tar.xz && \
    tar -xvf cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive.tar.xz && \
    mv cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive/include/* /usr/local/cuda/include/ && \
    mv cudnn-${CUDNN_ARCH}-8.9.5.30_cuda12-archive/lib/* /usr/local/cuda/lib64/


RUN add-apt-repository ppa:deadsnakes/ppa && apt-get update
RUN apt-get install -y python3.11 python3.11-distutils python-is-python3 python3-pip python3.11-dev

RUN pip install sounddevice pydub

RUN pip install TTS

WORKDIR /temp

# Install ROS1
COPY .ci/install_ros1.sh /temp/install_ros1.sh
RUN chmod +x /temp/install_ros1.sh && \
    /temp/install_ros1.sh && \
    cd .. && \
    rm -rf /temp


WORKDIR /catkin_ws

ENV PATH="/opt/ros/noetic/bin:${PATH}"

RUN apt-get install -y libsndfile1
RUN apt-get install -y espeak-ng
RUN pip install --upgrade protobuf
RUN apt-get install -y libportaudio2


RUN sudo apt-get install -y ros-noetic-audio-common


ENTRYPOINT [ "/bin/bash" ]