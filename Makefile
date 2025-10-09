.PHONY: docker-build docker-shell compose-build compose-up compose-down

IMAGE ?= se2_navigation:noetic
SERVICE ?= car_demo

docker-build:
	docker build --build-arg ROS_DISTRO=noetic -t $(IMAGE) .

docker-shell:
	xhost +local:root || true
	docker run --rm -it \
		-e DISPLAY=$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-e ROS_MASTER_URI=http://127.0.0.1:11311 \
		-e ROS_IP=127.0.0.1 \
		--network host \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		--device /dev/dri \
		--shm-size 1g \
		$(IMAGE) \
		bash

compose-build:
	docker compose build $(SERVICE)

compose-up:
	xhost +local:root || true
	docker compose up $(SERVICE)

compose-down:
	docker compose down
