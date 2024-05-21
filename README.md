# MiniStage

This program simulates a collective of robots. It is inspired by the [Stage](https://github.com/rtv/Stage/tree/master) simulator, but it's more lightweight, and it is customized for fast simulation of very simple robots.

============================================================

## Installation: 

To run MiniStage, you will need
* C++17 and a C++ compiler
* cmake
* FLTK
* OpenGL

I use the homebrew package manager on a M1 Mac. You may also need to install Xcode (Apple's developer toolset).

============================================================

## Build & run: 

After the packages are installed, build and run MiniStage using cmake. The steps below are written for Mac, but for a different operating system you can look up "how to compile and run a cmake project". 

* [Use cd in terminal to navigate to the folder you want to install in.]
* git clone git@github.com:lliu12/ministage.git
* mkdir ministage_build
* cd ministage_build
* cmake ../ministage  
* cmake --build .
* ./Run_MiniStage
