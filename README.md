# AUBO i7 — ROS 2 Humble + Gazebo + MoveIt 2 + FlexBE (тестовое задание)

Репозиторий: https://github.com/Voinarovsky/itmo_robot_test_ex

---

## 1. Запуск

```bash
git clone https://github.com/Voinarovsky/itmo_robot_test_ex.git
cd itmo_robot_test_ex
docker build -t aubo_ros2_test:latest .
docker compose up
```

```
[demo] 1/4 Запуск Gazebo (headless)...
[demo] 2/4 Активация joint_trajectory_controller...
[demo] 3/4 Запуск MoveIt (move_group)...
[demo] 4/4 Демо-движение через MoveIt...
```

### С RViz

```bash
xhost +local:docker
USE_RVIZ=true docker compose up
```

### Ручной режим

```bash
docker run --rm -it --network host --name aubo aubo_ros2_test:latest bash
```

Терминал 1 — Gazebo:
```bash
ros2 launch aubo_gazebo aubo_gazebo.launch.py \
     description_file:=aubo_gazebo.xacro gui:=false launch_rviz:=false
```

Терминал 2 (`docker exec -it aubo bash`) — контроллер и MoveIt:
```bash
ros2 control load_controller --set-state active joint_trajectory_controller
ros2 launch aubo_moveit_config aubo_moveit_sim.launch.py
```

Терминал 3 — FlexBE, запуск поведения `test7`:
```bash
ros2 launch flexbe_webui flexbe_full.launch.py use_sim_time:=true
```

## 2. Типы движений

| Тип | Реализация | Что делает |
|---|---|---|
| Конфигурационное пространство | `MoveGroupJointPlanExecuteState` (`flexbe_moveit2`) | планирование по углам суставов, OMPL |
| Операционное пространство | `CartesianMoveState` | прямая линия фланца через `/compute_cartesian_path` + `/execute_trajectory` |

Поведение `test7`: joint-space к целевой позе → Cartesian вверх на 0.15 м →
Cartesian вниз на 0.15 м → возврат в исходную точку.

`CartesianMoveState` берёт текущую позу `ee_link` из TF, строит смещённую цель,
запрашивает линейный путь с шагом 1 см и проверкой коллизий. Если
`fraction < min_fraction`  — движение не
исполняется, стейт уходит в `failed`.

---

## 3. Архитектура

```
┌───────────────────────────────────────────────────┐
│ FlexBE (flexbe_webui) — поведение test7           │
│   flexbe_moveit2 states    │
├───────────────────────────────────────────────────┤
│ MoveIt 2 / move_group                             │
│   OMPL, IK (KDL), коллизии, Cartesian path        │
│         ↓ action FollowJointTrajectory            │
├───────────────────────────────────────────────────┤
│ ros2_control                                      │
│   joint_state_broadcaster      (датчики)          │
│   joint_trajectory_controller  (привод)           │
├───────────────────────────────────────────────────┤
│ Gazebo 11 + плагин gazebo_ros2_control            │
│   физика, поднимает controller_manager            │
├───────────────────────────────────────────────────┤
│ URDF/xacro (робот + сцена) + SRDF (группа)        │
└───────────────────────────────────────────────────┘

## 4. Структура репозитория

```
Dockerfile              ROS 2 Humble + MoveIt + Gazebo + FlexBE + aubo
docker-compose.yml      запуск, проброс X11, ROS_DOMAIN_ID
docker/entrypoint.sh    source окружения ROS
docker/run_demo.sh      Gazebo → контроллеры → MoveIt → движение
patches/                доработки поверх апстримного aubo_ros2_driver
  aubo_description/urdf/scene.xacro                    сцена (стол + кубики)
  aubo_description/urdf/xacro/inc/aubo_gazebo.xacro    URDF для симуляции
  aubo_gazebo/launch/aubo_gazebo.launch.py             launch Gazebo
  aubo_moveit_config/launch/aubo_moveit_sim.launch.py  launch MoveIt
aubo_behaviors/
  aubo_flexbe_states/cartesian_move_state.py           стейт Cartesian-движения
  aubo_flexbe_behaviors/                               поведение test7
scripts/move_to_home.py     движение в конфигурационном пространстве
scripts/move_cartesian.py   движение по прямой
```
---
- `joint_trajectory_controller` не поднимается launch-файлом, активируется
  отдельно.
