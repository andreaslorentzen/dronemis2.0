#!/bin/bash

# INSTALL CLION PLUGIN && RIGHT CLICK + RUN. PISSE NEMT :) #

cd ~/workspaces/tum_simulator_ws/src/tum_simulator/cvg_sim_gazebo/launch/

rm dronemis_world.launch

wget https://www.dropbox.com/s/sxad5fryq4yygzs/dronemis_world.launch

cd ~/workspaces/tum_simulator_ws/src/tum_simulator/cvg_sim_gazebo/worlds/

rm dronemis_world.world

wget https://www.dropbox.com/s/gn9knl6swyw88j9/dronemis_world.world



# go to ~/workspaces/tum_simulator_ws/src/tum_simulator/cvg_sim_gazebo/worlds/dronemisworld.world and replace 'hippomormor' with your linux username..

