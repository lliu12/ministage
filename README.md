# MiniStage

![MiniStage Example Animation](ministage_example.gif)

This repository supports the paper "Noise-Enabled Goal Attainment in Crowded Collectives" by Lucy Liu, Justin Werfel, Federico Toschi, and L. Mahadevan. It contains data from the physical robot experiments (experiment_data folder) and code for running the simulations.

The MiniStage program simulates a collective of robots. It is inspired by the [Stage](https://github.com/rtv/Stage/tree/master) simulator.

============================================================

## Installation: 

To run MiniStage, you will need
* C++17 and a C++ compiler
* cmake
* FLTK
* OpenGL

============================================================

## Build & run: 

After the packages are installed, build and run MiniStage using cmake. The steps below are written for Mac, but for a different operating system you can look up "how to compile and run a cmake project". 

* [Use cd in terminal to navigate to the folder you want to install in.]
* git clone git@github.com:lliu12/ministage.git
* mkdir ministage_build
* cd ministage_build
* cmake ../ministage  
* cmake --build .
* ./run_ministage

The executables are as follows.
* ./run_ministage: Run simulation of robots with local sensing based controllers with an animated visualization. Starts paused; press 'P' to unpause. 
* ./run_astar: Run our implementation of the [Cooperative A*](https://ojs.aaai.org/index.php/AIIDE/article/view/18726) global planning program with an animated visualization. Starts paused; press 'P' to unpause. 
* ./get_ministage_results: Save data from simulations of robots with local sensing based controllers. Generates data used in Figure 2.
* ./get_astar_results: Save data from simulations of robots using the Cooperative A* planner. Generates data used in Figure 4.
* ./get_conditional_results: Generates data used in Figure 4.
* ./get_experimentmatch_results: Generates data used in Figure 3.

============================================================
