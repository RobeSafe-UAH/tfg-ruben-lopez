FROM ros:humble

SHELL ["/bin/bash", "-c"]

# Instalación de herramientas básicas y dependencias de MVSim
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-humble-rviz2 \
                       ros-humble-mvsim \
                       ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/* 

# Crear espacio de trabajo
RUN mkdir -p /sim_ws/src
WORKDIR /sim_ws

RUN source /opt/ros/humble/setup.bash && \
    colcon build

# Configuración del entorno
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /sim_ws/install/setup.bash" >> ~/.bashrc

# Alias
RUN echo "alias RUN_DEMO_1ROBOT='ros2 launch mvsim demo_1robot.launch.py'" >> ~/.bashrc
RUN echo "alias RUN_DEMO_WAREHOUSE='ros2 launch mvsim demo_warehouse.launch.py'" >> ~/.bashrc
RUN echo "alias RUN_TELEOP='ros2 run teleop_twist_keyboard teleop_twist_keyboard'" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]