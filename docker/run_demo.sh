#!/usr/bin/env bash
# Поднимает весь стек внутри одного контейнера:
#   Gazebo (headless) -> контроллеры -> MoveIt -> демо-движение
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

USE_RVIZ="${USE_RVIZ:-false}"

# сбрасываем кэш графа ros2cli — иначе проверки видят несуществующие сервисы
ros2 daemon stop >/dev/null 2>&1 || true

# сбрасываем кэш графа ros2cli — иначе проверки видят несуществующие сервисы
ros2 daemon stop >/dev/null 2>&1 || true

# сбрасываем кэш графа ros2cli — иначе проверки видят несуществующие сервисы
ros2 daemon stop >/dev/null 2>&1 || true

# сбрасываем кэш графа ros2cli — иначе проверки видят несуществующие сервисы
ros2 daemon stop >/dev/null 2>&1 || true

cleanup() {
  echo "[demo] завершаю дочерние процессы..."
  kill $(jobs -p) 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[demo] 1/4 Запуск Gazebo (headless)..."
ros2 launch aubo_gazebo aubo_gazebo.launch.py \
     description_file:=aubo_gazebo.xacro gui:=false launch_rviz:=false \
     > /tmp/gazebo.log 2>&1 &

echo "[demo] ждём controller_manager (на медленной машине до 5 минут)..."
for i in $(seq 1 150); do
  if timeout 10 ros2 control list_controllers >/dev/null 2>&1; then
    echo "[demo] controller_manager отвечает (попытка ${i}, ~$((i*3)) c)"
    break
  fi
  if (( i % 10 == 0 )); then echo "[demo]   ...ждём, прошло ~$((i*3)) c"; fi
  sleep 3
done
sleep 5
timeout 10 ros2 control list_controllers >/dev/null 2>&1 || {
  echo "[demo] FATAL: controller_manager не поднялся. Лог:"
  tail -40 /tmp/gazebo.log; exit 1; }

echo "[demo] 2/4 Активация joint_trajectory_controller..."
ros2 control load_controller --set-state active joint_trajectory_controller || true
ros2 control list_controllers

echo "[demo] 3/4 Запуск MoveIt (move_group)..."
ros2 launch aubo_moveit_config aubo_moveit_sim.launch.py \
     use_rviz:=${USE_RVIZ} > /tmp/moveit.log 2>&1 &

echo "[demo] ждём /move_action..."
for i in $(seq 1 90); do
  if ros2 action list 2>/dev/null | grep -q move_action; then break; fi
  if (( i % 15 == 0 )); then echo "[demo]   ...ждём move_group, прошло $((i*2)) c"; fi
  sleep 2
done
ros2 action list 2>/dev/null | grep -q move_action || {
  echo "[demo] FATAL: /move_action не появился. Лог:"
  tail -40 /tmp/moveit.log; exit 1; }
sleep 5

echo "[demo] 4/4 Демо-движение через MoveIt..."
python3 /ros2_ws/scripts/move_to_home.py

echo "[demo] Готово. Стек продолжает работать, Ctrl+C для выхода."
wait
