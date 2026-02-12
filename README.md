# mvsim_docker

---

Este repositorio almacena una imagen de docker que contiene el simulador MVSIM (https://github.com/MRPT/mvsim) y se lanza con GPU.

---

## ¡ATENCIÓN!

La carpeta 'src/' contiene el paquete 'rbsf_mvsim' con el trabajo desarrollado por **Fernando Villa Castell**

---

## MANUAL DE USO

Primero, se abre una terminal y se accede al directorio donde se desee clonar el repositorio. Una vez en dicho repositorio, ejecutar:

~~~bash
git clone https://github.com/fvilla12/mvsim_docker
cd mvsim_docker
~~~

La carpeta 'mvsim_docker' se organiza como:

- mvsim_docker
    -
src/
    -
docker-compose.yml
    -
Dockerfile
    -
Makefile
    -
README.md (Este texto)


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
Una vez termine el proceso ejecutamos:
~~~bash
make up
~~~
Terminado este último proceso, se abre una nueva terminal (situada, también, en 'mvsim_docker/') y se ejecuta:
~~~bash
make shell
~~~
Llegado este punto, todo está en preparado para ejecutar el simulador **MVSim**. 

Dentro de este contenedor podemos llamar al simulador a través de ROS2:
~~~bash
$ ros2 launch mvsim <nombre_del_mapa>.world.xml
~~~
> Si se deja el espacio *nombre_del_mapa.world.xml* en blanco y tabulamos, se muestra una lista con los mapas de demostración que integra el paquete 'mvsim'.

Para facilitar la ejecución, y ahorrar tiempo, se brindan los alias: RUN_DEMO_1ROBOT, RUN_DEMO_WAREHOUSE y RBSF_FORZAETH.
Los dos primeros lanzan los mundos *demo_1robot* y *demo_warehouse*, respectivamente. Mientras que el segundo lanza el mundo creado en el marco del laboratorio de investigación **ROBESAFE**.

---