FROM ros:galactic-ros-base-focal

# Install requirements
RUN apt-get update # cache-bump-1
RUN apt-get install -y nodejs npm python python3 python-is-python2
RUN apt-get install -y supervisor
RUN apt-get install -y libjansson-dev libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
RUN apt-get install -y ros-galactic-gazebo-ros-pkgs ros-galactic-xacro ros-galactic-robot-localization

# Drop privileges
RUN useradd --create-home -s /bin/bash cake
RUN mkdir /sim && chown cake:cake /sim
USER cake
WORKDIR /sim

# Fetch gzweb
RUN git clone https://github.com/osrf/gzweb

# Build gzweb
WORKDIR /sim/gzweb
RUN git checkout gzweb_1.4.1
RUN bash -c "git apply <(git show 9a24cfd340ffdcd0512bc257df3ff5e4d74b4691) # HTTPS SUPPORT"
RUN bash -c "source /usr/share/gazebo/setup.sh && npm run deploy --- -m local"

# Copy gzserver launcher
COPY --chown=cake:cake assets /sim/assets
RUN mkdir -p /sim/launcher
COPY --chown=cake:cake launcher /sim/launcher/launcher

# Config supervisord
COPY --chown=cake:cake supervisor.conf /sim/supervisord.conf
EXPOSE 8080

# Run
WORKDIR /sim
COPY --chown=cake:cake entrypoint.bash /sim/entrypoint.bash
## ENTRYPOINT sources /ros_entry into shell
CMD ["bash", "entrypoint.bash"]
