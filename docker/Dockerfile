# set the base image
FROM ubuntu:18.04

# author
MAINTAINER Hany Hamed

# extra metadata
LABEL version="0.3"
LABEL description="Learning-Stabilizing-Control-Policies-for-a-Tensegrity-Hopper-with-Augmented-Random-Search-paper docker image"

# update sources list
RUN apt-get update -y

# Install the required dependencies
RUN apt-get install -qy git nano g++ libglib2.0-dev curl freeglut3 freeglut3-dev cmake build-essential unzip g++-4.8 python python3-pip xvfb
RUN apt-get install -qy psmisc screen x11-xserver-utils lsof gnome-terminal
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.8
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 40 --slave /usr/bin/g++ g++ /usr/bin/g++-7
RUN pip3 install --upgrade setuptools
RUN pip3 install --upgrade pip
RUN pip3 install gym
RUN pip3 install tqdm
RUN pip3 install torch torchvision
RUN pip3 install tensorflow-gpu
RUN pip3 install ray[rllib]
RUN pip3 install ray[tune]
RUN pip3 install ray[debug]
RUN pip3 install pandas


RUN mkdir -p /home/Tensegrity \
    cd /home/Tensegrity

WORKDIR /home/Tensegrity
RUN git clone https://github.com/hany606/Tensegrity-Robotics.git --branch Learning-Stabilizing-Control-Policies-for-a-Tensegrity-Hopper-with-Augmented-Random-Search-paper

CMD ["/home/Tensegrity/Tensegrity-Robotics/bin/setup_helper.sh" ]

RUN Xvfb :3 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &> xvfb.log &
RUN export DISPLAY=:3

CMD ["/home/Tensegrity/Tensegrity-Robotics/bin/build.sh" ]

RUN export TENSEGRITY_HOME="/home/Tensegrity/Tensegrity-Robotics/"

