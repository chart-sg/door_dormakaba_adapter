FROM ghcr.io/junhaochng/rmf_deployment_template/builder-rmf

WORKDIR /opt/rmf/src

SHELL ["/bin/bash", "-c"]

ENV CONFIG_FILE="/opt/rmf/install/door_adapter_dormakaba/share/door_adapter_dormakaba/config.yaml"


RUN git clone https://github.com/sharp-rmf/door_adapter_dormakaba.git

WORKDIR /opt/rmf

RUN apt-get update && apt-get install -y python3-pip && \
  pip3 install requests urllib3 sockets jsons

# install
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
  apt-get update && rosdep install -y \
  --from-paths \
  src/door_adapter_dormakaba \
  --ignore-src \
  && rm -rf /var/lib/apt/lists/*

# Build
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
  colcon build --packages-select door_adapter_dormakaba

RUN sed -i '$iros2 run door_adapter_dormakaba door_adapter_dormakaba -c $CONFIG_FILE' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
