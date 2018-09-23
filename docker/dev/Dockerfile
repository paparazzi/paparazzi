FROM paparazziuav/pprz-dep
LABEL maintainer="felix.ruess@gmail.com"

# install some extra convenience packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    light-themes \
    terminator \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

ENV PULSE_SERVER /run/pulse/native

# add basic user
ENV USERNAME=pprz USER_ID=1000 USERGROUPS=sudo,dialout,plugdev
RUN useradd --shell /bin/bash -u $USER_ID -o -c "Paparazzi Docker user" -m $USERNAME \
    && usermod -aG $USERGROUPS $USERNAME

# set gtk theme
RUN echo include \"/usr/share/themes/Ambiance/gtk-2.0/gtkrc\" > /home/$USERNAME/.gtkrc-2.0


# handle permissions for docker volumes by dynamically changing the id of user pprz to LOCAL_USER_ID (default 1000)
# this uses https://github.com/tianon/gosu/
ENV GOSU_VERSION 1.9
RUN set -x \
    && wget -O /usr/local/bin/gosu "https://github.com/tianon/gosu/releases/download/$GOSU_VERSION/gosu-$(dpkg --print-architecture)" \
    && wget -O /usr/local/bin/gosu.asc "https://github.com/tianon/gosu/releases/download/$GOSU_VERSION/gosu-$(dpkg --print-architecture).asc" \
    && export GNUPGHOME="$(mktemp -d)" \
    && gpg --keyserver ha.pool.sks-keyservers.net --recv-keys B42F6819007F00F88E364FD4036A9C25BF357DD4 \
    && gpg --batch --verify /usr/local/bin/gosu.asc /usr/local/bin/gosu \
    && rm -r "$GNUPGHOME" /usr/local/bin/gosu.asc \
    && chmod +x /usr/local/bin/gosu \
    && gosu nobody true

COPY entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["bash"]
