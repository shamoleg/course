# Введение в GNU / Linux

Linux - семейство Unix-подобных операционных систем на базе ядра Linux, включающих тот или иной набор утилит и программ проекта GNU, и,  другие компоненты. Как и ядро Linux, системы на его основе как правило создаются и распространяются в соответствии с моделью разработки свободного и открытого программного обеспечения.

В нашем курсе мы будем исспользовать Ubuntu - дистрибутив Linux, основанный на Debian GNU/Linux.

Работа в графической оболочкке интуитивно понятна и походит на работу в операционных системах Windows, поэтому останавливаться на этом не будем.

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

Большенство пользователей привыкло работать на компьютере, кликая на иконки и выбирая нужные пункты меню. Однако есть и другой подход, который позволяет отдавать команды компьютеру, вводя их в специальную программу под названием терминал.

Для того чтобы открыть терминал неоюходимо кликнуть на эконку или зажать  клавиши `ctrl+alt+T`

## Основные команды которые потребуются нам для работы в контесте данного курса:

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

Cменить директорию

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

```

### **mv**

переместить файл

```console

```


### **echo**

Вывести строку

```console

```
