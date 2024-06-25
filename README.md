Проект по предмету "Программирование роботов". ИТМО 2024

## Требования

Перед началом работы убедитесь, что на вашем устройстве установлены следующие программы:
- Docker
- X server (для визуализации с Gazebo и RViz)

## Запуск контейнера

### Запуска контейнера на другом устройстве

1. Склонируйте репозиторий вашего проекта на целевое устройство:
```
git clone https://github.com/Demogam216/patrol_project.git
cd ~/catkin_ws/src
```

2. Сборка Docker контейнера:
```
docker build -t explore_patrol:latest .
```

3. Убедитесь, что X server разрешает подключения:
```
xhost +local:docker
```

4. Запуск контейнера с необходимыми параметрами для отображения GUI:
```
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    explore_patrol:latest
```

5. Внутри контейнера выполните следующие команды для настройки среды ROS:
```
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

```

### Запуск проекта

### Запуск исследования мира и зарисовки карты

1. В контейнере запустите исследование мира:
```
roslaunch explore_patrol auto_world_explore.launch world:=<название_мира> model:=<название_модели>
```

2. Откройте RViz на хост-машине, чтобы видеть процесс исследования и зарисовки карты:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=frontier_exploration
```

### Запуск патрулирования

1. Убедитесь, что в контейнере выполнены следующие команды:
```
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=<название_модели>
```

2. Запустите скрипт патрулирования:
```
./src/explore_patrol/scripts/start_patrol.sh <название_мира> <название модели>
```
где <название_мира> может быть clinic, cafe, или office;
где <название модели> может быть burger, waffle, waffle_pi.

### !!!На данный момент патрулирование реализовано только для мира "clinic"!!!
