# Симулирование показаний с датчиков

/TODO 

Перед добавлением датчика, необходимо создать сответсвущий элемент в URDF разметке робота, к ссылке которой будет привязан плагин датчика. Подробно о всех плагинах можно прочитать тут <http://gazebosim.org/tutorials?tut=ros_gzplugins>

## Инерционное измерительное устройство

IMU-модуль на 9 степеней свободы включает в себя три отдельных сенсора:

- **Акселерометр** для определения величины ускорения свободного падения по осям X, Y, Z.
- **Гироскоп** для определения угловой скорости вокруг собственных осей X, Y, Z.
- **Магнитометр** для определения углов между собственными осями сенсора X, Y, Z и силовыми линиями магнитного поля Земли.

Обрабатывая эти данные получаем ориентацию объекта в пространстве

Чтобы симулировать показания необходимо добавить плагин в
`gazebo_plugin.xacro`

Очень важно незабыть изменить тег `<bodyName>`, именнно от него будет расчитываться показания датчика.

```xml
<robot>
  ...
  <gazebo>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <alwaysOn>true</alwaysOn>
      <bodyName>imu_link</bodyName> 
      <topicName>imu</topicName>
      <serviceName>imu_service</serviceName>
      <gaussianNoise>0.0</gaussianNoise>
      <updateRate>20.0</updateRate>
    </plugin>
  </gazebo>
  ...
</robot>
```

## Лидар

Технология получения и обработки информации об удалённых объектах с помощью активных оптических систем, использующих явления поглощения и рассеяния света в оптически прозрачных средах.

Чтобы симулировать показания необходимо добавить плагин в
`gazebo_plugin.xacro`

```xml
<robot>
  ...
  <gazebo reference="lidar_link">
    <sensor type="gpu_ray" name="lidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_gpu_laser.so">
        <topicName>/rrbot/laser/scan</topicName>
        <frameName>lidar_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
  ...
</robot>
```

## Камера

```xml
<robot>
  ...
  <!-- camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
  ...
</robot>
```

## Запуск робота в симуляторе

Для удобного запуска необходимо создать загрузочный файл `coursebot_gazebo.launch` и прописать в нем условия появления робота на сцене.

```xml
<?xml version="1.0" encoding="UTF-8"?>

<launch>

    <arg name="x" default="0.0" />
    <arg name="y" default="0.0" />
    <arg name="z" default="0.2" />
    <arg name="urdf_robot_file" default="$(find coursebot_gazebo)/urdf/coursebot_gazebo.xacro" />
    <arg name="robot_name" default="coursebot" />

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

    <!-- загрузка URDF описисания -->
    <param name="robot_description" command="$(find xacro)/xacro.py '$(arg urdf_robot_file)'" />  
  


    <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    args="-urdf -x $(arg x) -y $(arg y) -z $(arg z)  -model $(arg robot_name) -param robot_description">
    <remap from="tf" to="gazebo_tf"/> 
    </node>
  

</launch>
```
