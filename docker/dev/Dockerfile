#From inside this folder
# docker build -t flixr/pprz-dev .
# docker run -t -i flixr/pprz-dev /bin/bash
# run with X11 forwarding:
# sudo docker run -t -i -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix flixr/pprz-dev /bin/bash

FROM flixr/pprz-dep
MAINTAINER Felix Ruess <felix.ruess@gmail.com>

ENV PULSE_SERVER /run/pulse/native

# add basic user
ENV USERNAME pprz
RUN useradd -m $USERNAME && \
	echo "$USERNAME:$USERNAME" | chpasswd && \
	usermod --shell /bin/bash $USERNAME && \
	usermod -aG sudo $USERNAME && \
	echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
	chmod 0440 /etc/sudoers.d/$USERNAME && \
	usermod --uid 1000 $USERNAME && \
	groupmod --gid 1000 $USERNAME

# change user
USER pprz

ENV HOME /home/$USERNAME
WORKDIR $HOME
