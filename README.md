# mvsim_docker
Este repositorio almacena una imagen de docker que contiene el simulador MVSIM (https://github.com/MRPT/mvsim) y se lanza con GPU.

## ¡ATENCIÓN!

La carpeta 'src/' tiene, a su vez, las carpetas: 'launch/' (contiene el mapa demo de e1 robot .py, .rviz y .xml), 'sensores/' (lidar2d y rgbd) y 'vehiculo/' (vehículo jackal .dae y .xml, también wheel.dae).
Estas 

## Proceso de instalación

Primero, en nuestra terminal principal clonamos el repositorio y compilamos el docker-compose:

~~~bash
git clone https://github.com/fvilla12/mvsim_docker
cd mvsim_docker

docker compose up -build
~~~

Luego, en otra terminal, abrimos el contenedor *mvsim_container*:

~~~bash
docker exec -it mvsim_container /bin/bash
~~~

Dentro de este contenedor lanzamos un mapa de un almacén predefinido con el comando *RUN_DEMO_WAREHOUSE*:

~~~bash
RUN_DEMO_WAREHOUSE
~~~
