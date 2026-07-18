# AUBO i7 — ROS 2 Humble + Gazebo + MoveIt 2 (тестовое задание)

Демо-стенд манипулятора **AUBO i7**: симуляция в Gazebo, планирование движений
через MoveIt 2, непустая сцена в URDF, всё упаковано в Docker.

Репозиторий: https://github.com/Voinarovsky/itmo_robot_test_ex

---

## 1. Запуск

Нужен только Docker (и `docker compose`). Ничего ставить в систему не надо.

```bash
git clone https://github.com/Voinarovsky/itmo_robot_test_ex.git
cd itmo_robot_test_ex
docker build -t aubo_ros2_test:latest .
docker compose up
```

Сборка образа занимает ~15–25 минут (тянется `ros-humble-moveit` и Gazebo).
Стек стартует headless и по шагам выводит прогресс:

```
[demo] 1/4 Запуск Gazebo (headless)...
[demo] 2/4 Активация joint_trajectory_controller...
[demo] 3/4 Запуск MoveIt (move_group)...
[demo] 4/4 Демо-движение через MoveIt...
```

### С RViz (нужен X11 на хосте, Linux)

```bash
xhost +local:docker
USE_RVIZ=true docker compose up
```

### Ручной режим (интерактивно, по терминалам)

```bash
docker run --rm -it --network host --name aubo aubo_ros2_test:latest bash
```

Терминал 1 (внутри контейнера):
```bash
ros2 launch aubo_gazebo aubo_gazebo.launch.py \
     description_file:=aubo_gazebo.xacro gui:=false launch_rviz:=false
```

Терминал 2 (`docker exec -it aubo bash`):
```bash
ros2 control load_controller --set-state active joint_trajectory_controller
ros2 launch aubo_moveit_config aubo_moveit_sim.launch.py
```

Терминал 3:
```bash
python3 /ros2_ws/scripts/move_to_home.py
```

---

## 2. Архитектура

```
┌───────────────────────────────────────────────────┐
│ FlexBE                                НЕ РЕАЛИЗОВАНО│
├───────────────────────────────────────────────────┤
│ MoveIt 2 / move_group                             │
│   OMPL-планировщик, IK (KDL), проверка коллизий   │
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
```

- Планировочная группа: `manipulator` (цепь `base_link` → `ee_link`)
- Kinematics solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`
- Модель: `aubo_i7`
- Все ноды запускаются с `use_sim_time: true` — обязательно при работе с
  Gazebo, иначе MoveIt виснет на «Didn't receive robot state»

---

## 3. Структура репозитория

```
Dockerfile              сборка образа: ROS 2 Humble + MoveIt + Gazebo + aubo
docker-compose.yml      запуск, проброс X11, ROS_DOMAIN_ID
docker/entrypoint.sh    source окружения ROS
docker/run_demo.sh      оркестрация: Gazebo → контроллеры → MoveIt → движение
patches/                доработки поверх апстримного aubo_ros2_driver
  aubo_description/urdf/scene.xacro                  сцена (стол + кубики)
  aubo_description/urdf/xacro/inc/aubo_gazebo.xacro  URDF для симуляции
  aubo_gazebo/launch/aubo_gazebo.launch.py           launch Gazebo
  aubo_moveit_config/launch/aubo_moveit_sim.launch.py  launch MoveIt
scripts/move_to_home.py демо-движение в конфигурационном пространстве
```

Апстримный репозиторий робота (https://github.com/AuboRobot/aubo_ros2_driver)
клонируется на этапе сборки образа и **не** вендорится в этот репозиторий —
поэтому в `patches/` видно ровно то, что было написано мной.
