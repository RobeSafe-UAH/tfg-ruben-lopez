# mvsim_docker

---

Este repositorio almacena una imagen de docker que contiene el simulador MVSIM (https://github.com/MRPT/mvsim) y se lanza con GPU.

---

## ¡ATENCIÓN!

La carpeta 'src/' contiene el paquete 'rbsf_mvsim' con el trabajo desarrollado por **Rubén López Ruiz**

---

## MANUAL DE USO

Primero, se abre una terminal y se accede al directorio donde se desee clonar el repositorio. Una vez en dicho repositorio, ejecutar:

~~~bash
git clone <nombre del repo>
cd mvsim_docker
~~~

La carpeta 'mvsim_docker' se organiza como:

- mvsim_docker
    - src/
    - docker-compose.yml
    - Dockerfile
    - Makefile
    - README.md (Este texto)

>Para facilitar el uso del repositorio, se hará uso del la herramienta **make**. Esta tiene diferentes usos, como: build, up down y shell. A continuación se expone el propósito de cada una:
>
>make build
>: Compila la imagen de Dockerfile
>make up
>: Arranca el contenedor
>make shell
>: Entra al contenedor en ejecución
>make down
>: Detiene los contedores 

Situados en la carpeta 'mvsim_docker' ejecutamos:
~~~bash
make build
~~~
Una vez terminado el proceso, se abre una nueva terminal (situada, también, en 'mvsim_docker/') y se ejecuta:
~~~bash
make shell
~~~
Llegado este punto, todo está en preparado para ejecutar el simulador **MVSim** a través del paquete de *MVSIM*-

Para poder utilizar el paquete **rbsf_mvsim** debe situarse en el directorio:
> /sim_ws/
Y ejecutar:
~~~bash
colcon build

source install/local_setup.bash
~~~
Una vez ejecutado, se ha de acceder al directorio:
> /sim_ws/src/

Aquí pueden ejecutarse las llamadas del paquete **rbsf_mvsim** que se enumeran a continicuación.

Dentro de este contenedor podemos llamar al simulador a través de ROS2:
~~~bash
ros2 launch mvsim <nombre_del_mapa>.world.xml
~~~
O para lanzar un launch concreto:

~~~bash
ros2 launch rbsf_mvsim <nombre_del_launch> world_file:=src/rbsf_mvsim/rbsf_mvsim/f1tenth_catalunya.world.xml
~~~

> Si se deja el espacio *nombre_del_mapa.world.xml* en blanco y tabulamos, se muestra una lista con los mapas de demostración que integra el paquete 'mvsim'.

Para facilitar la ejecución, y ahorrar tiempo, se brindan ciertos alias:
- Para lanzar mapas:
    - RUN_DEMO_1ROBOT
    - RUN_DEMO_WAREHOUSE
    - RBSF_FORZAETH
- Para grabar los topics de */r1* y los topis */tf* y */tf_static*:
    - GRABAR_TOPICS_R1 [-o <nombre_del_rosbag>]
- Para teleoperar el robot 1:
    - TELEOP_R1

Los dos primeros lanzan los mundos *demo_1robot* y *demo_warehouse*, respectivamente, que los suministra el paquete *mvsim*. Mientras que el tercero lanza el mundo creado en el marco del laboratorio de investigación **ROBESAFE**.

---

## Detalles TFG Rubén López

Actualmente para este proyecto se está usando el launch `rbsf_plan_control.launch.py` para lanzar todos los nodos necesarios en la simulación. Como mapa, se usa el `f1tenth_catalunya.world.xml`, pero están todas las lineas comentadas para no tener límites en el mapa mientras se hacen pruebas.

Los programas del proyecto están en la carpeta `src/rbsf_mvsim/rbsf_mvsim` que tiene la siguiente estructura:

- algorithms: Los algoritmos base, que no dependen de ros.
- metrics: Archivos de métricas generados automáticamente, y un programa para analizarlos (funcional pero faltan algunas graficas por generar).
- nodes: Nodos que se conectan a los topics de ros mediante subscribe o publish y llaman a los algoritmos para que realicen los cálculos pertinentes.

En los algoritmos, se encuentran los siguientes:
- **base_controller**: Clase base para los controladores. Implementa únicamente utilidades base y no puede ser usada por si misma.
- **base_path_optimizer**: Lo mismo para el optimizador de trayectorias.
- **centerline.csv / centerline_refined.csv**: Usado por los nodos de publicación de trayectorias para trabajar en simulación con un centerline generado de forma manual. Los centerlines reales se espera que sean publicados en un topic.
- **geometric_path_optimizer**: Optimizador de trayectorias que se usa actualmente. Su método `optimize_path` devuelve la trayectoria optimizada y el perfil de velocidad para dicha trayectoria.
- **kinematic_path_optimizer**: Optimizador de trayectorias basado en cinemática que se ha implementado parcialmente pero aún no se ha llegado a usar ni probar correctamente. Por ello, no se recomienda su uso.
- **none_path_optimizer**: Optimizador de trayectorias que no optimiza la trayectoria, pero sí calcula el perfil de velocidad. Se ha implementado para poder hacer pruebas con la centerline pura.
- **pure_pursuit_controller**: Controlador basado en el algoritmo de pure pursuit. Su método `calculate_control` devuelve el control a aplicar al vehículo para seguir la trayectoria dada.
- **pid_controller**: Controlador basado en un PID, se llama con el mismo método `calculate_control`.
- **map_controller**: Controlador basado en pure pursuit mejorado, usado por ETH. Se llama con el mismo método `calculate_control`.
- **mpc_controller**: Controlador basado en MPC, no completamente testeado en simulación, pero debería funcionar. Se llama con el mismo método `calculate_control`.
- **map_tools**: Actualmente solo es usado para obtener la centerline manual del archivo csv.
- **metrics**: Contiene todos los algoritmos y funciones usadas por el nodo de métricas para calcular las métricas de vuelta a partir de los datos de odometría.
- **speed_profiler y speed_controller**: Algoritmos legacy usados anteriormente para calcular y seguir el perfil de velocidad, actualmente implementado en el optimizador de trayectorias y el controlador respectivamente. Ya no se usan, pero se han dejado para mantener la historia del proyecto.

En la carpeta de nodos, además de todos los archivos de python de cada uno de los nodos, que no necesitan explicación, se destacan los siguientes archivos:
- **params.yaml**: Archivo de parámetros para los algoritmos y nodos, permite configurar el controlador que se usa, la velocidad, y muchos otros parámetros dependientes de cada algoritmo. Hay muchos otros archivos .yaml con distintas pruebas de optimización que se han hecho para cada controlador.
- **bayesian_tuner.py**: Programa pensado para lanzarse directamente con python, que implementa un optimizador bayesiano para optimizar los parámetros (`params.yaml`) de los algoritmos.
- **auto_param_tuner.py**: Optimizador de parámetros legacy, que no está correctamente adaptado para este simulador y utiliza un método de optimización muy básico, por lo que no se recomienda su uso. Se ha dejado para mantener la historia del proyecto.
- **centerline_generator.py**: Programa que refina una centerline manual. Actualmente no funciona correctamente debido al cambio en la simulación y la forma de publicar las coordenadas. En su lugar, se calcula la centerline directamente desde la imagen del mapa con el programa `extract_centerline.py`, situado en la carpeta superior junto con los archivos del mapa. En esa misma carpeta, también se encuentra el programa `map_image_generator.py` que genera la imagen del mapa a partir de un archivo .world.xml, pero los resultados son poco precisos y para el circuito actual se dispone de la imagen original.
- **iterative_tester.py**