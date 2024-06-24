# Используем базовый образ с полной установкой ROS Noetic Desktop
FROM osrf/ros:noetic-desktop-full

# Устанавливаем основные утилиты и Python пакеты
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    git

# Устанавливаем необходимые Python пакеты
RUN pip3 install rospkg catkin_pkg

# Инициализация и обновление rosdep
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update

# Установите зависимости для вашего проекта
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-explore-lite \
    ros-noetic-dwa-local-planner \
    && rm -rf /var/lib/apt/lists/*

# Создаем рабочую директорию
WORKDIR /root/catkin_ws

# Копирование вашего проекта в контейнер
COPY ./src /root/catkin_ws/src/

# Инициализация и сборка рабочего пространства catkin
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Установка точки входа в контейнер
ENTRYPOINT ["/bin/bash"]


