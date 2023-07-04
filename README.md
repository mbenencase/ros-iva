# ROS-IVA

ROS Intelligent Video Analytics is a simple collection of packages to deal with general purpose object detection tasks using ROS Kinetic.

# Docker Image

The Docker Image `ROS.dockerfile` is used as the development and test environment for ROS-IVA. We suggest to build as follows:

```
docker build -t ros-iva:latest -f ROS.dockerfile .
```

However, you can also build using `podman`

```
podman build -t ros-iva:latest -f ROS.dockerfile .
```

# How to Run

TODO
