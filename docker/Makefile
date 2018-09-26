all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build all pprz images"
	@echo "   1. make pull             - pull all pprz images"
	@echo "   1. make remove_images    - remove all pprz images"
	@echo "   2. make bash             - run bash on pprz-dev"
	@echo "   2. make test             - run make test on pprz-dev"
	@echo "   2. make terminator       - run terminator on pprz-dev"
	@echo "   2. make paparazzi        - run ./paparazzi center on pprz-dev"
	@echo "   2. make start            - run ./start.py on pprz-dev"
	@echo ""

build:
	docker build --tag=paparazziuav/pprz-dep dep/.
	docker build --tag=paparazziuav/pprz-dev dev/.
	docker build --tag=paparazziuav/pprz-ci ci/.

pull:
	docker pull paparazziuav/pprz-dep
	docker pull paparazziuav/pprz-dev
	docker pull paparazziuav/pprz-ci

remove_images:
	docker rmi -f paparazziuav/pprz-dep
	docker rmi -f paparazziuav/pprz-dev
	docker rmi -f paparazziuav/pprz-ci

bash terminator:
	@bash run.sh -i -t paparazziuav/pprz-dev $@

paparazzi:
	@bash run.sh -i -t paparazziuav/pprz-dev ./paparazzi

start:
	@bash run.sh -i -t paparazziuav/pprz-dev ./start.py

test:
	@bash run.sh -i -t paparazziuav/pprz-dev make test

.PHONY: all help build pull remove_images bash terminator paparazzi start test
