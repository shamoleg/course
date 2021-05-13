# tf

tf - это пакет, который позволяет пользователю отслеживать несколько координат с течением времени. tf поддерживает взаимосвязь между координатами в древовидной структуре, буферизованной во времени, и позволяет пользователю преобразовывать точки, векторы и т. д. между любыми двумя координатами в любой желаемый момент времени.

/TODO вставить картинку дерева tf

tf структуру  можно публиковать и читать програмно, но удобней создать визуальную модельробота с помощью URDF разметки.

URDF это пакет содержит ряд спецификаций XML для моделей роботов, датчиков, сцен и т.д.

Для начала создадим базу робота.

```xml
<?xml version="1.0"?>
<robot name="coursebot">
  <material name="blue">
      <color rgba="0 0 0.8 1"/>
  </material>
  <material name="red">
      <color rgba="1 0 0 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.05"/>
      </geometry>
       <material name="blue"/>
    </visual>
  </link>
</robot>
```

получим синий прямоугольник

![urdf_base](./image/urdf_base.png)

Покачто это просто прямоугольник, а так как мы планировали использовать данную модель в симуляторе, то необходимо определить момент энерции и грацницы соприкосновения. Для этого добавим теги в `base_link`

```xml
  ...
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.05"/>
      </geometry>
       <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0"
      izz="0.2"/>
    </inertial>
  </link>
  ...
```

так для расчета момента энерции воспользовались формулами

![urdf_base](./image/in_box.png)
![urdf_base](./image/in_cyl.png)

Добавим колесо

```xml
  ...
  <link name="front_right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.06" radius="0.06"/>
      </geometry>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <material name="red"/>
    </visual>
  </link>

  <joint name="base_to_front_right_wheel" type="fixed">
    <parent link="base_link"/>
    <child link="front_right_wheel"/>
    <origin xyz="0.16 0.15 -0.02"/>
  </joint>
  ...
```

По аналогии добавим остальные колёса и получим модель мобильного робота.


