all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build            - build all pprz images"
	@echo "   1. make pull             - pull all pprz images"
	@echo "   1. make clean            - remove all pprz images"
	@echo "   2. make bash             - run bash on pprz-dev"
	@echo "   2. make paparazzi        - run paparazzi center on pprz-dev"
	@echo ""

build:
	@docker build --tag=flixr/pprz-dep dep/.
	@docker build --tag=flixr/pprz-dev dev/.

pull:
	@docker pull flixr/pprz-dep
	@docker pull flixr/pprz-dev

clean:
	@docker rmi -f flixr/pprz-dep
	@docker rmi -f flixr/pprz-dev

bash:
	@bash run.sh -i -t flixr/pprz-dev bash

paparazzi:
	@bash run.sh -i -t flixr/pprz-dev ./paparazzi
