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

# Fetch gazebo models
RUN mkdir -p /root/.gazebo/models && \
    git clone --depth 1 -b master https://github.com/osrf/gazebo_models /root/.gazebo/models

# Make gazebo models accessible to gzweb
RUN rm -r /gzweb/http/client/assets/* && \
    ln -sf /root/.gazebo/models/* /gzweb/http/client/assets/ && \
    ls -d /gzweb/http/client/assets/*/ | xargs -I% mkdir -p %/meshes && \
    ls -d /gzweb/http/client/assets/*/ | xargs -I% bash -c "ln -s %/materials/textures/* %/meshes/"
RUN bash -c "source /usr/share/gazebo/setup.sh && npm run deploy --- -m local"

COPY assets /assets
COPY launcher /launcher

# Config supervisord
COPY supervisor.conf /etc/supervisor/supervisord.conf
EXPOSE 8080

CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
