#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This script includes all the steps of transfering an OpenSim model to Mujoco.

Created on Tue Jul 27 23:19:13 2021

@author: hwang

"""
from myoconverter.O2MPipeline import O2MPipeline

# define pipline configurations
kwargs = {}  # define kwargs inputs
kwargs['convert_steps'] = [1, 2, 3]    # All three steps selected
kwargs['muscle_list'] = None           # No specific muscle selected, optimize all of them
kwargs['osim_data_overwrite'] = True   # Overwrite the Osim model state files
kwargs['conversion'] = True            # Yes, perform 'Cvt#' process
kwargs['validation'] = True            # Yes, perform 'Vlt#' process
kwargs['speedy'] = True                # Do not reduce the checking notes to increase speed
kwargs['generate_pdf'] = True          # Do not generate validation pdf report
kwargs['add_ground_geom'] = True       # Add ground to the model
kwargs['treat_as_normal_path_point'] = False    # Using constraints to represent moving and conditional path points


############### Tug of War with Two Muscles ################
osim_file = './models/osim/TugOfWar/tugofwar.osim'
geometry_folder = './models/osim/TugOfWar/Geometry'
# output_folder = './models/mjc/TugOfWar'
output_folder = './models/mjc_speedy/TugOfWar'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

############### Simple Arm 2 DoFs 6 Muscles ################ 
osim_file = './models/osim/Arm26/arm26.osim'
geometry_folder = './models/osim/Arm26/Geometry'
# output_folder = './models/mjc/Arm26'
output_folder = './models/mjc_speedy/Arm26'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

############### Single Leg 6 DoFs 9 Muscles ################
osim_file = './models/osim/Leg6Dof9Musc/leg6dof9musc.osim'
geometry_folder = './models/osim/Leg6Dof9Musc/Geometry'
# output_folder = './models/mjc/Leg6Dof9Musc'
output_folder = './models/mjc_speedy/Leg6Dof9Musc'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

############### Simple 2D Gait 10 DoFs 18 Muscles ##############
osim_file = './models/osim/Gait10dof18musc/gait10dof18musc.osim'
geometry_folder = './models/osim/Gait10dof18musc/Geometry'
# output_folder = './models/mjc/Gait10dof18musc'
output_folder = './models/mjc_speedy/Gait10dof18musc'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

############### Simple 3D Gait 23 DoFs 54 Muscles ################
osim_file = './models/osim/Gait2354Simbody/gait2354.osim'
geometry_folder = './models/osim/Gait2354Simbody/Geometry'
# output_folder = './models/mjc/Gait2354Simbody'
output_folder = './models/mjc_speedy/Gait2354Simbody'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

############### 3D Gait Model with Simple Arms ################
osim_file = './models/osim/FullBody3D/fullbody.osim'
geometry_folder = './models/osim/FullBody3D/Geometry'
# output_folder = './models/mjc/FullBody3D'
output_folder = './models/mjc_speedy/FullBody3D'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

############### 6D Neck model ################
osim_file = './models/osim/Neck6D/neck6d.osim'
geometry_folder = './models/osim/Neck6D/Geometry'
# output_folder = './models/mjc/Neck6D'
output_folder = './models/mjc_speedy/Neck6D'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

############## Wrist and Hand Model ################
osim_file = './models/osim/WristHandModel/wristhand.osim'
geometry_folder = './models/osim/WristHandModel/Geometry'
# output_folder = './models/mjc/WristHandModel'
output_folder = './models/mjc_speedy/WristHandModel'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)
