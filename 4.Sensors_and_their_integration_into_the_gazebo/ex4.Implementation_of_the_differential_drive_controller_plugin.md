# Реализация плагина differential_drive_controller для работы с мобильной платформой в симуляторе

Для работы с симулятором необходимо добавить специальные плагины для работы с каждым датчиком и приводом.

Создадим файл `gazebo_plugin.xacro` где добавим и настроим плагин дифференциального привода, с указанием джоинтов правого и левого колеса. Особое внимание стоит уделить названиям, они должны совпадать с описанием робота из прошлого урока.

```console
cd ~/catkin_ws/src/bot_description/urdf
touch gazebo_plugin.xacro
```

Добавим описание плагина в фаил

```xml
<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro" > 
    <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <legacyMode>false</legacyMode>
            <alwaysOn>true</alwaysOn>
            <publishWheelTF>true</publishWheelTF>
            <publishTf>1</publishTf>
            <publishWheelJointState>true</publishWheelJointState>
            <updateRate>20.0</updateRate>
            <!-- изменить на сочленения правого и левого колеса -->
            <leftJoint>front_left_wheel_joint</leftJoint>
            <rightJoint>front_right_wheel_joint</rightJoint>
            <wheelSeparation>0.08</wheelSeparation>
            <wheelDiameter>0.2</wheelDiameter>
            <wheelAcceleration>1</wheelAcceleration>
            <torque>20</torque>
            <commandTopic>/cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_footprint</robotBaseFrame>
        </plugin>
    </gazebo>
</robot>
```

<div style="page-break-before:always;">
</div>

Также создадим файл `bot_gazebo.xacro`, где будет импортироваться описание робота и написанные нами плагины.

```console
cd ~/catkin_ws/src/bot_description/urdf
touch gazebo_plugin.xacro
```

```xml
<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro" >
  
  <xacro:include filename="$(find bot_description)/urdf/bot_description.xacro"/>
  <xacro:include filename="$(find bot_description)/urdf/gazebo_plugin.xacro"/>

</robot>
```

Для загрузки в ROS необходимо создать загрзочный файл `bot_gazebo.launch` в папке `launch` текущего пакета

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>

    <arg name="x" default="0.0" />
    <arg name="y" default="0.0" />
    <arg name="z" default="0.2" />
    <arg name="urdf_robot_file" default="$(find bot_description)/urdf/bot_gazebo.xacro" />
    <arg name="robot_name" default="bot" />
    
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <param name="robot_description" command="$(find xacro)/xacro.py '$(arg urdf_robot_file)'" />

    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    args="-urdf -x $(arg x) -y $(arg y) -z $(arg z)  -model $(arg robot_name) -param robot_description"/>

</launch>
```

Запустим пустую симуляцию, для этого в окне терминала введем команду

```console
roslaunch gazebo_ros empty_world.launch
```

Заспавним нашего робота

```console
roslaunch bot_description bot_gazebo.launch
```

Наш робот должен появиться в центре координат

Для контроля скорости запустим ноду управления с клавиатуры, которая шлёт угловую и линейную скорости в топик /cmd_vel. Для начало установим пакет с данной нодой

```console
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

Теперь запустим её командой

```console
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Передвижения осуществляется нажатием соответствующих клавиш в открытом окне терминала. Для работы необходимо выбрать английскую раскладку

```console
Moving around:
   u    i    o
   j    k    l
   m    ,    .
```

Если все сделано правильно, получившееся мобильная платформа должна передвигаться плавно с корректной линейной и угловой скоростью.
