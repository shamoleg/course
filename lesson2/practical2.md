
# Создание пакета, публиканта и подписчика

## Создание рабочей области

Для создания союственнного пакета необходимо создать рабочую область, где будут храниться исходный код пакетов.

```console
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

Чтобы добавить рабочую область в среду ROS, вам необходимо выполнить скрипт лежащий в файле `~/catkin_ws/devel/setup.bash`. Для удобство добавим его выполнение в файл /.basgrc и запустим командами:

```console
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Пакеты хранятся в папе src, перейдем в эту папку.

```console
cd ~/catkin_ws/src
```

Создадим пакет с помощью команды `catkin_create_pkg <имя_пакета> [зависимость1] [зависимость2] [зависимость3]`

```console
catkin_create_pkg start_pkg std_msgs rospy roscpp
```

Соберем пакет

```console
cd ~/catkin_ws
catkin_make
```

В итоге должно получиться дерево файлов

```console
catkin_ws/               -- рабочая область
  build/
    ...
  devel/
    ...
  src/                   
    CMakeLists.txt       -- фаил сборки рабочей области
    start_pkg/           -- папка созданного пакета
      CMakeLists.txt     -- фаил сборки пакета
      package.xml        -- манифест
```

## Создание публиканта

Измените каталог на пакет start_pkg

```console
roscd start_pkg
```

Сначала давайте создадим папку 'scripts' для хранения наших скриптов Python

```console
mkdir scripts
cd scripts
```

Создадим файл публиканта.

```console
touch talker.py   
chmod +x talker.py
```

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```


Создадим скрипт подписчика.

```console
roscd start_pkg/scripts/
touch listener.py
chmod +x listener.py
```

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```
