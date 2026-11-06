# Jonas en la laptop: ROS 2 Jazzy (la misma versión que la Raspi) para simular y para ver el robot real en RViz.
# Liviano a propósito: ros-base + RViz, sin Gazebo ni desktop-full.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-aiohttp python3-pygame python3-numpy \
        python3-colcon-common-extensions \
        mesa-utils libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# El usuario "ubuntu" (uid 1000) de la imagen coincide con el de la laptop: los archivos creados no quedan como root
RUN usermod -aG video ubuntu \
    && mkdir -p /home/ubuntu/ws/build /home/ubuntu/ws/install /home/ubuntu/ws/log \
    && chown -R ubuntu:ubuntu /home/ubuntu/ws
COPY entrypoint.sh /entrypoint.sh
USER ubuntu
WORKDIR /home/ubuntu/ws
ENV ROS_DOMAIN_ID=7
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
