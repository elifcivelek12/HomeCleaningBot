# Temel İmaj: ROS 2 Humble (Desktop Full versiyonu, içinde Gazebo ile geliyormuş)
FROM osrf/ros:humble-desktop-full

# 1. Gerekli Linux Araçlarını ve ROS Paketlerini Yükle
RUN apt-get update && apt-get install -y \
    nano \
    python3-pip \
    ros-humble-ros-gz \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# 2. Çalışma Alanını (Workspace) Oluştur
WORKDIR /root/colcon_ws

# 3. Terminal her açıldığında ROS'u otomatik kaynak (source) yap
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
# Eğer içeride derleme yaptıysak onu da kaynak yap
RUN echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

# 4. Giriş noktası (Container açık kalsın diye bash başlatıyoruz)
CMD ["/bin/bash"]
