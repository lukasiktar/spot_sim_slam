# Person-detection image: ROS 2 Humble + YOLO (Ultralytics)
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions wget \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /tmp/wheels && cd /tmp/wheels && \
    curl -4 --tls-max 1.2 -fSL \
      -o torch-2.6.0+cpu-cp310-cp310-linux_x86_64.whl \
      "https://download.pytorch.org/whl/cpu/torch-2.6.0%2Bcpu-cp310-cp310-linux_x86_64.whl" && \
    curl -4 --tls-max 1.2 -fSL \
      -o torchvision-0.21.0+cpu-cp310-cp310-linux_x86_64.whl \
      "https://download.pytorch.org/whl/cpu/torchvision-0.21.0%2Bcpu-cp310-cp310-linux_x86_64.whl" && \
    pip3 install --no-cache-dir --timeout 300 --retries 8 ./*.whl && \
    cd / && rm -rf /tmp/wheels && \
    pip3 install --no-cache-dir --timeout 300 --retries 8 \
      ultralytics "opencv-python<5"


RUN pip3 install --no-cache-dir "numpy<2"

RUN mkdir -p /models && \
    wget -q -O /models/yolov8n.pt \
      https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt && \
    python3 -c "from ultralytics import YOLO; YOLO('/models/yolov8n.pt')"

ENV YOLO_CONFIG_DIR=/tmp/yolo
ENV MPLCONFIGDIR=/tmp/mpl

COPY spot_search /ws/src/spot_search

WORKDIR /ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
