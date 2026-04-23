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
                       '~nros-humble-rqt*' \
                       ros-$ROS_DISTRO-rviz2 \
                       ros-$ROS_DISTRO-mvsim \
                       ros-$ROS_DISTRO-teleop-twist-keyboard \
                       ros-$ROS_DISTRO-slam-toolbox \
    && pip install pandas \
                   matplotlib \
                   "numpy<2" \
                   scikit-learn \
                   norfair \
                   optuna \
                   rosbags \
    && rm -rf /var/lib/apt/lists/* 

# Crear espacio de trabajo
RUN mkdir -p /sim_ws/src && chown -R $USER_UID:$USER_GID /sim_ws

WORKDIR /sim_ws
USER $USERNAME

RUN source /opt/ros/humble/setup.bash && \
    colcon build

RUN source install/local_setup.bash

# Configuración del entorno
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /sim_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /sim_ws/install/local_setup.bash" >> ~/.bashrc

# Alias
## Para lanzar el mapa DEMO_1ROBOT de MVSIM
RUN echo "alias RUN_DEMO_1ROBOT='ros2 launch mvsim demo_1robot.launch.py'" >> ~/.bashrc
## Para lanzar el mapa DEMO_WAREHOUSE de MVSIM
RUN echo "alias RUN_DEMO_WAREHOUSE='ros2 launch mvsim demo_warehouse.launch.py'" >> ~/.bashrc
## Para lanzar el mapa RBSF_FORZAETH de RBSF_MVSIM
RUN echo "alias RBSF_FORZAETH='ros2 launch rbsf_mvsim rbsf_forzaeth.launch.py'" >> ~/.bashrc
## Para grabar un rosbag con los topics que se publiquen en el namespace '/r1'. AÑADIR (-o <fichero_salida>)
RUN echo 'alias GRABAR_TOPICS_R1="ros2 bag record -e \"/r1/*|/perception/*\""' >> ~/.bashrc
## Compilar el paquete 'rbsf_mvsim'
RUN echo 'alias COMPILAR="colcon build --symlink-install --packages-select rbsf_mvsim"' >> ~/.bashrc
## Para lanzar el nodo de teleop para escribir en el topic '/r1/cmd_vel'
RUN echo 'alias TELEOP_R1="ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/r1/cmd_vel"' >> ~/.bashrc
## Separación de dominio de ROS para no saturar la red de comunicación
RUN echo 'export ROS_DOMAIN_ID=4' >> ~/.bashrc

WORKDIR /sim_ws/src