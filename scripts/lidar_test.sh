#!bin/bash

#Установка зависимостей
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev python-rosinstall python-rosinstall-generator python-wstool build-essential git

# Перeйдем в папку ~/catkin_ws/src
cd ~/catkin_ws/src

#установка git
sudo apt install git -y

# Клонирование репозитория с помощью команды:
git clone https://github.com/Slamtec/rplidar_ros.git

# Переход в корневую директорию workspace:
cd ~/catkin_ws

#Выполним сборку пакета с помощью команды:
catkin_make

# Выполним активацию окружения:
source devel/setup.bash

#Для постоянного добавления в окружение введем команду:
echo "source ~/catkin_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc