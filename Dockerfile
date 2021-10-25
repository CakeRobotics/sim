FROM ros:galactic-ros-base-focal
RUN apt-get update
RUN apt-get install -y ros-galactic-gazebo-ros-pkgs
RUN apt-get install -y supervisor

# Build gzweb
RUN apt-get install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
WORKDIR /
RUN git clone https://github.com/osrf/gzweb
WORKDIR /gzweb
RUN git checkout gzweb_1.4.1
RUN apt-get install -y nodejs npm
RUN apt-get install -y python
RUN apt-get install -y python3 python-is-python2
RUN bash -c "source /usr/share/gazebo/setup.sh && npm run deploy --- -m local"

# Copy gzserver launcher
RUN apt-get install -y \
    ros-galactic-xacro \
    ros-galactic-robot-localization
COPY assets /assets
COPY launcher /launcher/launcher

# Create unprivileged user
RUN useradd --create-home -s /bin/bash cake && \
    chown -R cake:cake /launcher /gzweb

# Config supervisord
COPY supervisor.conf /etc/supervisor/supervisord.conf
EXPOSE 8080

# Run
COPY entrypoint.bash /entrypoint.bash
USER cake
RUN touch /launcher/logs /gzweb/logs
# ENTRYPOINT sources /ros_entry into shell
CMD ["bash", "/entrypoint.bash"]
