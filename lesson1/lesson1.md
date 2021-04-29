# Введение в GNU / Linux

Linux - семейство Unix-подобных операционных систем на базе ядра Linux, включающих тот или иной набор утилит и программ проекта GNU, и,  другие компоненты. Как и ядро Linux, системы на его основе как правило создаются и распространяются в соответствии с моделью разработки свободного и открытого программного обеспечения.

В нашем курсе мы будем использовать Ubuntu - дистрибутив Linux, основанный на Debian GNU/Linux.

Работа в графической оболочке интуитивно понятна и походит на работу в операционных системах Windows, поэтому останавливаться на этом не будем.

## Структура файловой системы Ubuntu

Ubuntu поддерживает стандарт FHS(File System Hierarchy Standard), описывающий какая информация должна находится в том или ином месте «дерева». Ниже приведена таблица основных директорий с кратким описанием .

| Директория  | Описание |
| ------------| ---------|
| /      | Корневая директория, содержащая всю файловую иерархию. |
| /home/ | Содержит домашние директории пользователей.|
| /opt/  | Дополнительное программное обеспечение. |
| /dev/  | Основные файлы устройств системы. |
| /bin/  | Основные системные утилиты. |
| /boot/ | Загрузочные файлы (в том числе файлы загрузчика, ядро и т.д.).|

## Terminal

Большинство пользователей привыкло работать на компьютере, кликая на иконки и выбирая нужные пункты меню. Однако есть и другой подход, который позволяет отдавать команды компьютеру, вводя их в специальную программу под названием терминал.

Для того чтобы открыть терминал необходимо кликнуть на иконку или нажать сочетание клавиш `ctrl+alt+T`

## Основные команды которые потребуются в контексте данного курса

### **pwd** (print working directory)

Вывести текущую директорию

```console
olegsham@course:~$ pwd
/home/olegsham
```

### **ls** (list directory content)

Вывести содержимое директории. Если не передать путь, то выведется содержимое текущей директории.

```console
olegsham@course:~$ ls
catkin_ws      Desktop    education          Music     Public   Templates
CLionProjects  Documents  exampl.txt         Pictures  res.txt  Video
desktop        Downloads  genicam_xml_cache  program   snap     youbot
olegsham@course:~$ ls catkin_ws/
build  devel  src
olegsham@course:~$ ls -l catkin_ws/
итого 12
drwxr-xr-x 20 olegsham olegsham 4096 мар 11 15:36 build
drwxr-xr-x  6 olegsham olegsham 4096 мар 10 02:18 devel
drwxr-xr-x  9 olegsham olegsham 4096 мар 11 18:12 src

```

### **cd**

Сменить директорию

```console
olegsham@course:~$ pwd
/home/olegsham
olegsham@course:~$ cd catkin_ws/src/
olegsham@course:~/catkin_ws/src$ pwd
/home/olegsham/catkin_ws/src
```

### **mkdir**

создать директорию.

```console
olegsham@course:~/catkin_ws/src$ ls
CMakeLists.txt
olegsham@course:~/catkin_ws/src$ mkdir courcebot
olegsham@course:~/catkin_ws/src$ ls
CMakeLists.txt  courcebot
```

### **touch**

создать файл.

```console
olegsham@course:~/catkin_ws/src$ ls
CMakeLists.txt  courcebot
olegsham@course:~/catkin_ws/src$ touch readme.txt
olegsham@course:~/catkin_ws/src$ ls
CMakeLists.txt  courcebot  readme.txt
```

### **сp**

копировать файл

```console
olegsham@course:~/catkin_ws/src$ ls
CMakeLists.txt  courcebot  readme.txt
olegsham@course:~/catkin_ws/src$ cp readme.txt courcebot/
olegsham@course:~/catkin_ws/src$ ls courcebot/
readme.txt
```

### **rm**

удалить файл

```console
olegsham@course:~/catkin_ws/src/courcebot$ ls
readme.txt
olegsham@course:~/catkin_ws/src/courcebot$ rm readme.txt 
olegsham@course:~/catkin_ws/src/courcebot$ ls
```

### **mv**

переместить файл

```console
olegsham@course:~/catkin_ws/src$ mv readme.txt courcebot/
olegsham@course:~/catkin_ws/src$ ls
CMakeLists.txt  courcebot
olegsham@course:~/catkin_ws/src$ ls courcebot/
readme.txt
```

### **echo**

Вывести строку

```console
olegsham@course:~/catkin_ws/src$ echo hi
hi
```

### **chmod**

Изменение прав доступа

```console

```

## Горячии главиши для работы в командной строке

| Сочетание клавиш  | Описание |
| ------------| ---------|
|**🠕, 🠗** | Перемещение по командам введенным ранее.|
|**🠖, 🠔** | Перемещение по команде.|
|**Ctrl + A** | Перевод коретки в начало.|
|**Ctrl + E** | Перевод коретки в конец.|
|**Ctrl + Shift + C** | Копирование выделленного текста.|
|**Ctrl + Shift + V** | Вставка текста.|
|**Tab** | Подстановка |
|**Tab, Tab** | Показ возможныч постановок |

## Специальные симовы

| Символ | Описание |
| ------------| ---------|
|**.** | Текущее состояние |
|**..** | Директория на уровень выше |
|**~** | Домашняя директория($HOME) |

## BASH

Bourne-Again SHell, усовершенствованная и модернизированная вариация командной оболочки Bourne shell. Одна из наиболее популярных современных разновидностей командной оболочки UNIX.

При входе в систему для всех пользователей, выполняются команды, содержащиеся в файлах /etc/profile и $HOME/.profile.

Файл /etc/profile позволяет администратору системы выполнить обслуживающие действия для всех пользователей.

Файл ~/.profile используется для установки нужных конкретному пользователю переменных окружения и характеристик терминала.

Когда запускается интерактивная оболочка, которая не является оболочкой входа в систему, bash читает и выполняет команды ~/.bashrc

## Настройка рабочего окружения для разработки мобильного робота и взаимодействия с сенсорами

ROS (Robot Operating System) — Операционная система для роботов — это экосистема для программирования роботов, предоставляющий функциональность для распределенной работы. ROS был первоначально разработан в 2007 году под названием switchyard в Лаборатории Искусственного Интеллекта Стэнфордского Университета.

ROS обеспечивает стандартные службы операционной системы, такие как: аппаратную абстракцию, низкоуровневый контроль устройств, реализацию часто используемых функций, передачу сообщений между процессами, и управление пакетами. ROS основан на архитектуре графов, где обработка данных происходит в узлах, которые могут получать и передавать сообщения между собой. Библиотека ориентирована на Unix-подобные системы (Ubuntu Linux включен в список «поддерживаемых», в то время как другие варианты, такие как Fedora и Mac OS X, считаются «экспериментальными»).

## Установка ROS

Добавим репозиторий пакетов ROS.

```console
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Настроим ключ

```console
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Обновим список доступных репозиториев и обновим установленные пакеты

```console
sudo apt-get update 
sudo apt-get upgrade
```

Для обучения лучше всего установить полную версию системы содержащую: ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators and 2D/3D perception

```console
sudo apt install ros-melodic-desktop-full
```

Настройка рабочих параметров ROS, происходит через установку переменных окружения. Для  автоматического экспорта при запуске интерактивной оболочки bash(открытия терминала) добавим команду в ~/.bashrc

```console
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Для создания и управления собственными рабочими пространствами ROS существуют различные инструменты , которые распространяются отдельно. Их тоже необходимо установить.

```console
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

Прежде чем вы сможете использовать многие инструменты ROS, вам нужно будет инициализировать rosdep. rosdep позволяет легко устанавливать системные зависимости для исходного кода, который вы хотите скомпилировать, и требуется для запуска некоторых основных компонентов в ROS.

```console
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

На этом установка ROS завершена.

## Установка VScode

Для разработки нам также потребуется редактор исходного кода. Один из самых удобных инструментов является VScod, для ехо установке нам потребуется менеджер пакетов snap.

```console
sudo apt install snapd
```

Далее идет непостредственно установка VScode

```console
sudo snap install --classic code
```

Для запуска достаточно в ввести терменале

```console
code
```

Для работы с ROS необходимо установить расширения

- Python by Microsoft (ms-python.python)
- C/C++ by Microsoft (ms-vscode.cpptools)
- CMake by twxs (twxs.cmake)
