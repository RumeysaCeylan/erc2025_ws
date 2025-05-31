FROM ros:jazzy-ros-core


RUN apt update && apt install -y \
    ros-jazzy-v4l2-camera \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/app

COPY ros_clients.py .

CMD ["bash"]

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
