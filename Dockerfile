FROM osrf/ros:humble-desktop

COPY include/ccd /usr/include/ccd

ARG USERNAME=airlab
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME 

WORKDIR /home/airlab/BOW

USER $USERNAME
