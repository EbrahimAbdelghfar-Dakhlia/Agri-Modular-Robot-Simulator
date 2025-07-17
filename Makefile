# Usage: make [command]
SHELL:=/bin/bash
WORKSPACE=$(shell pwd)
.PHONY: build # to avoid error
setup_docker:
	docker build -t jazzy_image:latest -f docker/Dockerfile .
up_container:
	docker compose -f docker/compose.yaml up
exec_container:
	docker compose -f docker/compose.yaml exec mppi_jazzy /bin/bash
