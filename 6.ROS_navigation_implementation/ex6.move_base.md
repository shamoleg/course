# Управление мобильной платформой

В данной работе вам предстоит построить карту лабиринта используя пакет ROS move_base задавая координату цели для мобильной платформы, при этом визуализируя данные с датчиков в rviz.

В данной работе можно использовать мобильную платформу, разрабатываемого вами в предыдущих упражнениях, либо воспользоваться готовой платформой. Просмотреть пакет можно по ссылке <https://github.com/shamoleg/bot_description>

А также пакет с настройками навигационного стека. Просмотреть пакет можно по ссылке <https://github.com/shamoleg/bot_navigation>

## Установка пакета с навигационным стеком

Установите пакеты необходимые для работы навигации

```console
sudo apt-get install ros-noetic-dwa-local-planner
sudo apt-get install ros-noetic-map-server
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-actionlib
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-amcl
```

Откройте директорию с пакетами вашей рабочей области и скачайте пакет с настроенной навигацией:

```console
cd ~/catkin_ws/src
git clone https://github.com/shamoleg/bot_navigation
```

Перейдите в воркспейс и соберите проект:

```console
cd ~/catkin_ws
catkin_make
```

Проект готов к запуску

## Запуск навигационого стека

Для запуска мира с лабиринтом вам необходимо в окне терминала выполнить команду:

```console
roslaunch bot_description eazy_maze_world.launch
```

Для спавна мобильной платформы в новом окне терминала выполнить команду:

```console
roslaunch bot_description bot_gazebo.launch
```

Запустите картографирование slam, выполнив команду в новом окне терминала:

```console
roslaunch bot_navigation nav_gmapping.launch
```

Для визуализаци воспользуетесь rviz, для его открытия выполните команду в новом окне терминала:

```console
rviz
```

Добавьте следующие элементы визуализации нажав кнопку `Add`: RobotModel, Path, Map, LaserScan

Перемещая робота по помещению, постройте карту. Для сохранения карты перейдите в папку, где они располагаются, и отправьте команду на сохранение.

```console
rosrun map_server map_saver -f mymap
```
