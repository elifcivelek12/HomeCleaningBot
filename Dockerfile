# Temel İmaj: ROS 2 Humble
FROM ros:humble-ros-base

# 1. Gerekli Temel Araçları Yükle (Curl, Gpg vs.)
RUN apt-get update && apt-get install -y \
    curl gnupg lsb-release git \
    python3-tk \
    python3-pip \
    mesa-utils \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# 2. Gazebo Harmonic Deposunu Ekle (Çünkü Humble varsayılan olarak Fortress kurar, biz Harmonic istiyoruz)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. Paketleri Yükle 
RUN apt-get update && apt-get install -y \
    nano \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-tf2-ros \
    ros-humble-joint-state-publisher-gui \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# 4. Çalışma Alanı Ayarları
WORKDIR /root/colcon_ws
COPY ./src /root/colcon_ws/src

# 5. Projeyi Derle
RUN . /opt/ros/humble/setup.sh && colcon build

# 6. Kaynak Dosyaları (Source)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

# 7. Varsayılan Başlatma Komutu (Bringup Launch)
CMD ["/bin/bash", "-c", "source /root/colcon_ws/install/setup.bash && ros2 launch home_cleaning_robot bringup.launch.py"]
