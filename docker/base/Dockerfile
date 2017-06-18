# Ubuntu base image
FROM ubuntu:16.04

# Dockerfile maintainer
MAINTAINER Kazuya Otani, <kazuotani14@gmail.com>

USER root

# Install prerequisites
RUN apt-get update && apt-get install -y wget
RUN apt-get update && apt-get install -y \
                       build-essential \
                       mesa-common-dev \
                       apt-utils \
                       cmake \
                       make \
                       locate \
                       git \
                       curl \
                       software-properties-common \
                       sudo

RUN apt-get update && apt-get install -y --fix-missing curl

RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get update
RUN apt-get install -y gcc-5 g++-5
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5

# Add basic user
ENV USERNAME camerauser
RUN useradd -m $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && \
    usermod --shell /bin/bash $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    # Replace 1000 with your user/group id
    usermod  --uid 1000 $USERNAME && \
    groupmod --gid 1000 $USERNAME


# Change user
USER camerauser
