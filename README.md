## A PX4 Integrated Framework for Modeling and Controlling Multicopters with Tiltable Rotors

__abstract__ This repo presents a general framework for multicopters equipped with tiltable rotors (tilting multicopters). Differently from classical flat multicopters, tilting multicopters can be fully actuated systems able to decouple position and attitude control. The proposed framework has been trans-parently integrated into the widely used PX4 control stack, an open-source controller for ground and aerial systems, to fully exploit its high-level interfaces and functionalities and, at the same time, simplify the creation of new devices with tilting propellers. Simulation tools have been also added to the PX4 simulation framework, based on its Software-In-The-Loop (SITL) system and a set of simulated experiments in a dynamic robotic simulator have been carried out to demonstrate the effectiveness of this system. 

## Article 
The description of the firmware architecture, the integration with the standard PX4 control stack, as well as its integration with a PixHawk autopilot is described in the following article:

``Salvatore Marcellini, Jonathan Cacace, Vincenzo Lippiello, "A PX4 Integrated Framework for Modeling and Controlling Multicopters with Tiltable Rotors", submitted to the 2023 International Conference on Unmanned Aircraft System (ICUAS ’23)  June 6 – 9, 2023  Warsaw, Poland``

This work is currently under review

## Get the firmware 
This repo will be filled with all necessary data and documentation after that tha associated article will be accepted


## Video
https://youtu.be/N61GHj4W_II

## How to use
Clone the repository with submodules
$ git clone --recurse-submodule https://github.com/prisma-lab/PX4_tilting_multicopters.git

# Run the simulation
For omnidirectional tilting drone
$ make px4_sitl gazebo_NDT_tilting

For one-tilt tilting drone
$make px4_sitl gazebo_baby_k
