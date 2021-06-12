# Управление мобильной платформой

Перед тем как перейти непосредственно к управлению, необходимо созать класс, где бут прописаны все топики на которые будет потписана система управления и топики, куда будет отпраляться управляющие сигналы.

Созададим новый пакет в `~/catkin_ws/src` и добавим необходимые зависимости

```console
catkin_create_pkg coursebot_control rospy std_msgs
```

Создадим папку `scripts` где создадим файл с классом управления роботом `robot_control.py` . ВАЖНО, в названияъ файлов использовать строчные символы, а также не забывать про разрешение на выполнение.

Ниже приведен пример реализации, необходимо дополнить методы

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time


class RobotControl():

    def __init__(self, robot_name="turtlebot"):
        rospy.init_node('robot_control_node')

        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()        

        self.laser_subscriber = rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)
        self.laser_msg = LaserScan() 
        
        self.rate = rospy.Rate(30)


    def publish_once_in_cmd_vel(self):
        self.vel_publisher.publish(self.cmd)

    def laser_callback(self, msg):
        self.laser_msg = msg

    def get_laser(self, pos):
        return self.laser_msg.ranges[pos]

    def get_laser_full(self):
        return self.laser_msg.ranges

    def stop_robot(self):
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0
        self.publish_once_in_cmd_vel()

    def move_forward(self):

        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        self.publish_once_in_cmd_vel()


    def turn(self, clockwise, speed, time):
      # добавить реализацию аналогично предидущем методам



if __name__ == '__main__':
    
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()

    except rospy.ROSInterruptException:
        pass
```