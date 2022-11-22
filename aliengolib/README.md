# Aliengolib

## Overview
Aliengolib is a library providing the implementation of Aliengo's kinematics and dynamics. It is based on [Robotlib](git@gitlab.advr.iit.it:dls-lab/robotlib.git), a modular and generic robot software interface, and on [RobCoGen](https://robcogenteam.bitbucket.io/), an open source code generator that generates an efficient implementation of rigid body dynamics and kinematics algorithms for articulated robots. It was implemented within the project [ANT](https://www.dfki.de/en/web/research/projects-and-publications/project/ant/) as a glue code interfacing Robotlib with Aliengo kinematics and dynamics. For more detail about the project, read the paper [Towards a generic navigation and locomotion control system for legged space exploration](https://az659834.vo.msecnd.net/eventsairwesteuprod/production-atpi-public/7bfd9084af194454bbc98fa9c9d7b648).

Aliengolib extends the Robotlib interface to implement the particular morphology, kinematics, and dynamics of Aliengo. Thanks to polymorphisms, it is possible to use the interface to access the deepest robot specific implementation of functions declared or defined in Robotlib. In Aliengolib, the implementation of such functions is based on RobCoGen.

This library can be considered as a glue code and, to understand the rationale behind it, you can read the Robotlib [Overview-TODO]() and [Usage-TODO]() sections.

**Authors in alphabetic order**: Gianluca Cerilli, Geoff Fink and Marco Marchitto

## Installation
### Dependencies
Aliengolib has been developed and tested on a x86_64 version of Ubuntu 20.04 (Focal Fossa). The dependencies for building and installing the library are the following:

**CMake** (3.8.0 is the minimum version for C++17 standard) - You can download the chosen version and install it through

    wget https://cmake.org/files/v3.<X>/cmake-3.<X>.<X>-Linux-x86_64.tar.gz
    tar xf cmake-3.<X>.<X>-Linux-x86_64.tar.gz
    export PATH="$PATH:/home/<path where you extracted cmake>/cmake-3.<X>.<X>-Linux-x86_64/bin"

You just need to substitue \<X> with the chosen CMake version.

**Eigen3**

    sudo apt install libeigen3-dev

**GTest**

    sudo apt install libgtest-dev

<!--TODO - urdf installation-->

### Building
To build Aliengolib, clone the latest version of this repository and compile the package using

    git clone git@gitlab.advr.iit.it:dls-lab/aliengo-commons.git

    cd aliengo-commons/aliengolib

    mkdir build

    cd build

    cmake .. -DCMAKE_BUILD_TYPE=Release

    make install

If you get the error *CMAKE_MAKE_PROGRAM is not set.* when executing the `cmake` command, you might need to do

    sudo apt install build-essential

## Usage
Aliengolib was developed with the idea of having a robot dependent shared library to be lodaded at run time, to guarantee modularity to a control framework based on Robotlib. Its usage is tightly related to how Robotlib is used, because all the Aliengolib functions and variables are accessed through the generic interface Robotlib, exploiting polymorphisms. Therefore, to understand how to use Aliengolib you can read the section [Usage-TODO]() of Robotlib.

For other type of usage, you can have a look at the tests in the *tests* folder.

## Documentation
The Aliengolib documentation is written using Doxygen. To generate the documentation go in the folder *doc* and execute the following command

    doxygen aliengolib_doxygen.conf

Latex and html files will be generated according to the instructions provided in the configuration file aliengolib_doxygen.conf. 

To access to the html documentation, just double click on the file *index.html* stored in the folder *doc/html*: it will open the file in your browser.

## Tests
The tests are based on GoogleTests: the Google's C++ test framework.

To run tests

    mkdir build

    cd build

    cmake .. -DBUILD_TESTING=On

    make install

    make check

## Issues

<!-- TODO -->
<!-- Clean issue tracker and make some internal developments (like Agile tasks) private. Put there only known issues for public -->
You can look for known issues, report bugs and ask for features implementation at the [issue tracker](https://gitlab.advr.iit.it/dls-lab/aliengo-commons/-/issues).
