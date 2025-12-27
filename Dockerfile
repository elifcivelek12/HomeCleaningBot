# Temel İmaj: ROS 2 Humble
FROM ros:humble-ros-base

# 1. Gerekli Temel Araçları Yükle (Curl, Gpg vs.)
RUN apt-get update && apt-get install -y curl gnupg lsb-release

# 2. Gazebo Harmonic Deposunu Ekle (Çünkü Humble varsayılan olarak Fortress kurar, biz Harmonic istiyoruz)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 3. Paketleri Yükle (Artık 'ros-humble-ros-gzharmonic' kuruyoruz)
RUN apt-get update && apt-get install -y \
    nano \
    python3-pip \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# 4. Çalışma Alanı Ayarları
WORKDIR /root/colcon_ws

# 5. Kaynak Dosyaları (Source)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
