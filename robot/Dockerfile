FROM ros:jazzy-ros-core

# Sistem gereksinimleri
RUN apt update && apt install -y \
    python3-opencv \
    ros-jazzy-v4l2-camera \
    ros-jazzy-cv-bridge

# Çalışma dizini
WORKDIR /root/app

# Local dosyaları kopyala
COPY ros_commands.py .

# Varsayılan komut (isteğe bağlı)
CMD ["bash"]

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]