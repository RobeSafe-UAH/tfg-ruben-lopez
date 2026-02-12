FROM ros:humble

SHELL ["/bin/bash", "-c"]

ARG USERNAME
ARG USER_UID
ARG USER_GID

# Crear usuario y grupo (si el ID no existe)
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Instalación de herramientas básicas y dependencias de MVSim
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
# Librerías necesarias para (fvilla12)
                       libglib2.0-bin \
                       libgl1-mesa-glx \
                       libgl1-mesa-dri \
#
                       ros-humble-rviz2 \
                       ros-humble-mvsim \
                       ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/* 

# Crear espacio de trabajo
RUN mkdir -p /sim_ws/src && chown -R $USER_UID:$USER_GID /sim_ws

WORKDIR /sim_ws
USER $USERNAME

RUN source /opt/ros/humble/setup.bash && \
    colcon build

# Configuración del entorno
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /sim_ws/install/setup.bash" >> ~/.bashrc

# Alias
RUN echo "alias RUN_DEMO_1ROBOT='ros2 launch mvsim demo_1robot.launch.py'" >> ~/.bashrc
RUN echo "alias RUN_DEMO_WAREHOUSE='ros2 launch mvsim demo_warehouse.launch.py'" >> ~/.bashrc
RUN echo "alias RBSF_FORZAETH='mvsim launch /sim_ws/src/rbsf_mvsim/rbsf_mvsim/rbsf_forzaeth.world.xml'" >> ~/.bashrc

WORKDIR /sim_ws/src