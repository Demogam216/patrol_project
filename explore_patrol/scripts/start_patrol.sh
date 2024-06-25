#!/bin/bash

if [ -z "$1" ] || [ -z "$2" ]; then
  echo "Usage: $0 [clinic|cafe|office] [burger|waffle|waffle_pi]"
  exit 1
fi

WORLD=$1
MODEL=$2

# Устанавливаем переменную модели робота
export TURTLEBOT3_MODEL=$MODEL

# Найти файлы с использованием команды find
PATROL_SCRIPT=$(find ~/catkin_ws/src -type f -name "${WORLD}_patrol.py")

# Запускаем Gazebo с выбранным миром
roslaunch $(find ~/catkin_ws/src -name custom_world.launch) world:=$WORLD &
GAZEBO_PID=$!

# Ожидаем, пока Gazebo загрузится
sleep 120

# Запускаем навигацию с соответствующей картой
roslaunch $(find ~/catkin_ws/src -name patrol_navigation.launch) map:=$WORLD &
NAV_PID=$!

# Ожидание завершения запуска навигации
sleep 10

# Публикуем начальную позу робота, направленного на 0 градусов
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {
    seq: 0,
    stamp: {secs: 0, nsecs: 0},
    frame_id: 'map'
  },
  pose: {
    pose: {
      position: {x: 1.0, y: 3.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0, 0, 0, 0, 0,
                 0, 0.25, 0, 0, 0, 0,
                 0, 0, 0.25, 0, 0, 0,
                 0, 0, 0, 0.06853891945200942, 0, 0,
                 0, 0, 0, 0, 0.06853891945200942, 0,
                 0, 0, 0, 0, 0, 0.06853891945200942]
  }
}" &

# Ожидание выполнения команды
sleep 1

# Публикуем начальную точку цели
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{
  header: {
    seq: 0,
    stamp: {secs: 0, nsecs: 0},
    frame_id: 'map'
  },
  pose: {
    position: {x: 1.1, y: 3.1, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}" &

# Ожидание выполнения команды
sleep 1

# Запуск Python скрипта патрулирования
python3 $PATROL_SCRIPT &

# Ожидание завершения процессов
wait $GAZEBO_PID
wait $NAV_PID
