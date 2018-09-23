FROM paparazziuav/pprz-dep
LABEL maintainer="felix.ruess@gmail.com"

RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    libgazebo7-dev \
    && rm -rf /var/lib/apt/lists/*
