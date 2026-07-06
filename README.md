# GlueCode

## Overview
GlueCode is a library providing the implementation of kinematics and dynamics of robots, by using Pinocchio library. It is extending [Robotlib](https://github.com/iit-DLSLab/robotlib), a modular and generic robot software interface. It was implemented within the project [ANT](https://www.dfki.de/en/web/research/projects-and-publications/project/ant/) as a glue code interfacing Robotlib with Aliengo kinematics and dynamics. For more detail about the project, read the paper [Towards a generic navigation and locomotion control system for legged space exploration](https://az659834.vo.msecnd.net/eventsairwesteuprod/production-atpi-public/7bfd9084af194454bbc98fa9c9d7b648).

This library can be considered as a glue code and, to understand the rationale behind it, you can read the Robotlib [Overview](https://github.com/iit-DLSLab/robotlib#overview) and [Usage](https://github.com/iit-DLSLab/robotlib#usage) sections.

<br />

**Authors in alphabetical order**: Gianluca Cerilli, Geoff Fink and Marco Marchitto

## Installation
**Robotlib**

Follow the instructions [here](https://github.com/iit-DLSLab/Robotlib#installation) to install Robotlib.

### Building
### Open docker image
Open the docker image you have downloaded when installing Robotlib

```bash
docker run -it --rm \
  --name dls_container \
  --hostname docker \
  --gpus all \
  --privileged \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e DLS=2 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD:$PWD" \
  -w "$PWD" \
  ghcr.io/iit-dlslab/dls2-dev:latest \
  /bin/bash
```
### Build
To build GlueCode, clone the latest version of this repository and compile the package using

    git clone git@gitlab.advr.iit.it:dls-lab/aliengo-commons.git

    cd aliengo-commons/glue_code

    mkdir build

    cd build

    cmake .. -DCMAKE_BUILD_TYPE=Release -DDLS_INSTALL_MODEL_DIR=/usr/lib/dls2/robots -DDLS_INSTALL_MODEL_HEADER_DIR=/usr/lib/dls2/robots

    make

    sudo make install

## Usage
GlueCode was developed with the idea of having a robot dependent shared library to be lodaded at run time, to guarantee modularity to a control framework based on Robotlib. Its usage is tightly related to how Robotlib is used, because all the GlueCode functions and variables are accessed through the generic interface Robotlib, exploiting polymorphisms. Therefore, to understand how to use GlueCode you can read the section [Usage](https://gitlab.advr.iit.it/dls-lab/robotlib/-/tree/master#usage) of Robotlib.
