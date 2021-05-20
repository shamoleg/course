# gazebo

Gazebo - это симулятор трехмерной робототехники с открытым исходным кодом, так же является самым популярным среди разработчиков ros.

## Реализация плагинов gazebo для работы с мобильной платформой в симуляторе

Для работы с симулятором необходимо добавить спецальные плагины для работы с каждым датчиком и приводом.

Создадим новый пакет, где будет хранится описание для работы в симуляторе.

```console
cd ~/catkin_ws/src
catkin_create_pkg coursebot_gazebo
```

Создадим файл `gazebo_plugin.xacro` где добавим и настроим плагин дифференциального привода, с указанием джоинтов правого и левого колеса. Особое внимание стоит уделить названиям, они должны совпадать с описанием робота из прошлого урока.

```xml
<robot name="coursebot" xmlns:xacro="http://www.ros.org/wiki/xacro" >
  
    <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <legacyMode>false</legacyMode>
            <alwaysOn>true</alwaysOn>
            <publishWheelTF>true</publishWheelTF>
            <publishTf>1</publishTf>
            <publishWheelJointState>true</publishWheelJointState>
            <updateRate>60.0</updateRate>
            <leftJoint>front_left_wheel_joint</leftJoint>
            <rightJoint>front_right_wheel_joint</rightJoint>
            <wheelSeparation>0.08</wheelSeparation>
            <wheelDiameter>0.2</wheelDiameter>
            <wheelAcceleration>1</wheelAcceleration>
            <torque>20</torque>
            <commandTopic>/cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_link</robotBaseFrame>
        </plugin>
    </gazebo>
</robot>
```

Тут же опредилим колеса как элемент трансмиссии, для каждого колеса создадим специальный тег

```xml
...
  <transmission name="left_wheel_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="front_left_wheel_joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>  
    </joint>
    <actuator name="left_wheel_actuator">
      <mechanicalReduction>7</mechanicalReduction>
      <hardwareInterface>VelocityJointInterface</hardwareInterface>
    </actuator>
  </transmission>
...
```

Так же создадим файл `coursebot_gazebo.xacro`, где будет импортироваться модель робота и написанные нами плагины.

```xml
<robot name="coursebot" xmlns:xacro="http://www.ros.org/wiki/xacro" >
  
  <xacro:include filename="$(find coursebot_description)/urdf/coursebot.xacro"/>
  <xacro:include filename="$(find coursebot_gazebo)/urdf/gazebo_plugin.xacro"/>

</robot>
```

Для загрузки в ROS необходимо создать загрзочный файл `coursebot_gazebo.launch`

```xml
<?xml version="1.0" encoding="UTF-8"?>

<launch>

    <arg name="x" default="0.0" />
    <arg name="y" default="0.0" />
    <arg name="z" default="0.2" />
    <arg name="urdf_robot_file" default="$(find coursebot_gazebo)/urdf/coursebot_gazebo.xacro" />
    <arg name="robot_name" default="coursebot" />
    
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <param name="robot_description" command="$(find xacro)/xacro.py '$(arg urdf_robot_file)'" />


    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    args="-urdf -x $(arg x) -y $(arg y) -z $(arg z)  -model $(arg robot_name) -param robot_description"/>

    <!-- <rosparam file="$(find coursebot_gazebo)/config/diff_drive.yaml" command="load" /> -->

</launch>
```

Запустим пустую симуляцию, для этого в окне терминала введем команду

```console
roslaunch gazebo_ros empty_world.launch
```

Заспавним нашего робота

```console
roslaunch coursebot_gazebo coursebot_gazebo.launch
```

Наш робот должен появиться в центре координат

Для контроля скорости запустим ноду управления с клавиатуры, котора шлёт угловую и линейную скорости в топик /cmd_vel

```console
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
