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
RUN set -eux \
    && apt-get update \
    && apt-get install -y gosu \
    && rm -rf /var/lib/apt/lists/* \
    && gosu nobody true

COPY entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["bash"]
