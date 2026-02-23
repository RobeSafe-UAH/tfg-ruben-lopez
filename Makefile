# Capturar datos del usuario actual
export USER_NAME := $(shell whoami)
export USER_ID   := $(shell id -u)
export USER_GID  := $(shell id -g)

.PHONY: build up down shell

# Construir la imagen pasando los argumentos
build:
	docker-compose build --build-arg USERNAME=$(USER_NAME) \
                         --build-arg USER_UID=$(USER_ID) \
                         --build-arg USER_GID=$(USER_GID)
	docker compose up
# Levantar el contenedor
up:
	docker compose up

# Entrar al contenedor en ejecución
shell:
	docker exec -it mvsim_container /bin/bash

# Detener contenedores
down:
	docker-compose down