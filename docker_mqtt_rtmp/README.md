# docker_mqtt_rtmp

This folder contains a **lightweight Docker image** that bundles **Mosquitto MQTT broker** (port **1883**) and an **NGINX RTMP server** (port **1935**) into a single container.  
It is the fastest way to provide telemetry/control messaging and video streaming services required by the *sanbot_ros* bridge.

---
## :bookmark_tabs: Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Build the Image](#build-the-image)
4. [Run the Container](#run-the-container)
5. [Container Management](#container-management)
6. [License](#license)

---
## Prerequisites

| Host | Software | Minimum version | Purpose |
|------|----------|-----------------|---------|
| Ubuntu / Debian | Docker Engine | 20.10 | Container runtime |

> If Docker is not installed follow the steps below *(tested on Ubuntu 20.04)*:
>
> ```bash
> # Install prerequisites
> sudo apt-get update
> sudo apt-get install -y ca-certificates curl gnupg lsb-release
>
> # Add Docker’s official GPG key
> sudo mkdir -p /etc/apt/keyrings
> curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
>   sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
>
> # Add the stable repository (replace $(lsb_release -cs) if needed)
> echo \
>   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
>   https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | \
>   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
>
> # Install Docker Engine + CLI + plugins
> sudo apt-get update
> sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
>   docker-buildx-plugin docker-compose-plugin
>
> # Start & enable the service
> sudo systemctl enable --now docker
>
> # (Optional) Run docker without sudo
> sudo usermod -aG docker $USER
> newgrp docker
> ```

---
## Installation

Clone the repository (if you haven't already) and move into this directory:

```bash
cd ~/catkin_ws/src/sanbot-ros/docker_mqtt_rtmp
```

---
## Build the Image

Build the image locally with a descriptive tag:

```bash
docker build -t sanbot_mqtt_rtmp .
```

First build takes longer, subsequent builds are cached.

---
## Run the Container

Launch the container in **detached** mode and make it restart automatically at boot:

```bash
docker run -d --name sanbot_mqtt_rtmp \
  --restart unless-stopped \
  -p 1883:1883 \   # MQTT broker
  -p 1935:1935 \   # RTMP server
  sanbot_mqtt_rtmp
```

Endpoints exposed by the container:

* MQTT broker – `mqtt://<host-ip>:1883`
* RTMP server – `rtmp://<host-ip>:1935/live/stream`

Once the container is running you can launch the **ROS–MQTT bridge** with:

```bash
roslaunch sanbot_ros bridge.launch
```

---
## Container Management

| Action | Command |
|--------|---------|
| View running containers | `docker ps` |
| Stop container | `docker stop sanbot_mqtt_rtmp` |
| Remove container | `docker rm sanbot_mqtt_rtmp` |
| View logs (follow) | `docker logs -f sanbot_mqtt_rtmp` |
| Rebuild image & recreate | `docker rm -f sanbot_mqtt_rtmp && docker build -t sanbot_mqtt_rtmp . && docker run …` |

---
## License

This image is part of the **sanbot-ros** project and distributed under the **MIT License**. See the root [`LICENSE`](../LICENSE) file for details. 