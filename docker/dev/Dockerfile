#From inside this folder
# docker build -t flixr/pprz-dev .
# docker run -t -i flixr/pprz-dev /bin/bash
# run with X11 forwarding:
# sudo docker run -t -i -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix flixr/pprz-dev /bin/bash

FROM flixr/pprz-dep
MAINTAINER Felix Ruess <felix.ruess@gmail.com>

# Set the env variable DEBIAN_FRONTEND to noninteractive
ENV DEBIAN_FRONTEND noninteractive

# install some extra convenience packages
RUN apt-get update && apt-get install -y \
    light-themes \
    terminator

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV PULSE_SERVER /run/pulse/native

# add basic user
ENV USERNAME pprz
ENV USERGROUPS sudo,dialout
RUN useradd -m $USERNAME && \
	echo "$USERNAME:$USERNAME" | chpasswd && \
	usermod --shell /bin/bash $USERNAME && \
	usermod -aG $USERGROUPS $USERNAME && \
	echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
	chmod 0440 /etc/sudoers.d/$USERNAME && \
	usermod --uid 1000 $USERNAME && \
	groupmod --gid 1000 $USERNAME

# change user
USER pprz

ENV HOME /home/$USERNAME
WORKDIR $HOME

# set gtk theme
RUN echo include \"/usr/share/themes/Ambiance/gtk-2.0/gtkrc\" > $HOME/.gtkrc-2.0
