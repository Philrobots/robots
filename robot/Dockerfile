FROM ros:melodic-ros-core

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-opencv \
        python3-setuptools \
        build-essential \
        poppler-utils \
        tesseract-ocr \
        libtesseract-dev \
        libleptonica-dev \
        iputils-ping \
    && ldconfig \
    && apt-get install -y --no-install-recommends \
        libsm6 \
        libxext6 \
    && apt-get clean

RUN mkdir -p /root/catkin_ws/src
COPY . /root/catkin_ws/src

WORKDIR "/root/catkin_ws"
COPY .pylintrc .
COPY requirements.txt .

RUN pip3 install --upgrade setuptools pip
RUN pip3 install -r requirements.txt
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_init_workspace /root/catkin_ws/src && catkin_make"

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
