# Работа с узлами через терминал

Перед началом работы следует запустить мастер roscore. Его следует запускать каждый раз при начале работы.
В командной строке введите команду

```console
$ roscore
```

Запустите новый узел с помощью команды rosrun. Для этого следует воспользоваться пакетом симулятора черепахи
turtlesim. В командной строке введите команду

```console
$ rosrun turtlesim turtlesim_node
```

Будет выведена информация о запущенном узле и одновременно появится графическое окно с черепахой.

![tortle](./image/urdf_tr.png)

<div style="page-break-before:always;">
</div>

Откройте новый терминал и выполните вывод списка активных узлов.

```console
$ rosnode list

/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

Если возникает необходимость запустить еще один узел turtlesim, то ROS не даст это сделать, так как в системе будет два узла с одинаковым именем, работа существующего узла будет прекращена, и будет создан новый узел.

Для создания еще одного узла с другим именем необходимо переопределить имя.

```console
$ rosrun turtlesim turtlesim_node __name:=my_turtle
```

Для прекращения работы узла необходимо в окне терминала нажать сочетание клавиш <nobr>`Ctrl + C`</nobr>

Одним из примеров управления является ввод с клавиатуры например нодой.

```console
$ rosrun turtlesim turtle_teleop_key
```

Открыв новое окно терминала и введя команду

```console
$ rqt_graph
```

Появится новое графическое окно с графом связей между
узлами и темой. В графе присутствует два узла: teleop_turtle и turtlesim, связанные через тему
/turtle1/cmd_vel. Узел teleop_turtle публикует данные в тему /turtle1/cmd_vel, а узел turtlesim подписан на
эту тему и получает данные.

![rqt_graph](./image/nodegraf.png)

Посмотреть, какие данные передаются через тему /turtle1/cmd_vel, можно с помощью команды

```console
$ rostopic echo /turtle1/cmd_vel
```

Данные будут выводится в терминал только при их обновлении. То есть необходимо начать движение черепашкой.

Если в данный момент открыть окно с графом связей между узлами и обновить содержимое окна, то будет видно, что появился еще один узел (rostopic_xxxx_xxхxx), подписанный на тему  /turtle1/cmd_vel

Oбмен информацией между узлами через темы происходит в виде сообщений. Сообщения должны иметь определенный тип, известный и узлу-издателю, и узлу-подписчику. Таким образом, тип темы определяется типом сообщений, которые в ней публикуются. Тип темы можно получить с помощью команды

```console
$ rostopic type /turtle1/cmd_vel

geometry_msgs/Twist
```

<div style="page-break-before:always;">
</div>

Полную информацию о теме /turtle1/cmd_vel можно получить с помощью команды

```console
$ rostopic info /turtle1/cmd_vel

Type: geometry_msgs/Twist
Publishers:
* /teleop_turtle (http://machine_name:35547/)
Subscribers:
* /turtlesim (http://machine_name:37930/)
* /rostopic_24776_1530557986993
(http://machine_name:41231/)
```

Тип темы по сути является типом сообщения, которое передается в этой теме. В данном случае тема использует тип сообщения geometry_msgs/Twist. Имя сообщения состоит из двух частей: имя_пакета/имя_типа. Таким образом, тема /turtle1/cmd_vel использует тип сообщения под именем.

Twist из пакета geometry_msgs. Информацию о типе сообщения можно получить с помощью команды rosmsg

```console
$ rosmsg show geometry_msgs/Twist

geometry_msgs/Vector3 linear
float64 x
float64 y
float64 z
geometry_msgs/Vector3 angular
float64 x
float64 y
float64 z
```

Сформируем сообщение для темы /turtle1/cmd_vel с помощью команды

```console
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

<div style="page-break-before:always;">
</div>

Видим что черепашка пройдя определенное расcтояние остановилась, это следствие того, что сообщение было опубликовано 1 раз, для постоянной отправки сообщений через <nobr>`rostopic pub`</nobr> добавить флаг -r (rate) со значением частоты отправки сообщений в герцах.

```console
$ rostopic pub -r 50 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 
```

Черепашка теперь не останавливается.

rqt_plot отображает временную диаграмму данных, опубликованных по темам. Здесь мы будем использовать rqt_plot для построения данных, публикуемых в теме /turtle1/pose . Запустите rqt_plot.

```console
$ rqt_plot
```