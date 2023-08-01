#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This script includes the test of transfering an OpenSim model to Mujoco.

Created on Tue Jul 27 23:19:13 2021

@author: hwang

"""
from myoconverter.O2MPipeline import O2MPipeline

# define pipline configurations
kwargs = {}  # define kwargs inputs
kwargs['convert_steps'] = [1, 2, 3]    # All three steps selected
kwargs['muscle_list'] = ["longissi_cerv_c4thx_L"]           # No specific muscle selected, optimize all of them
kwargs['osim_data_overwrite'] = True   # Overwrite the Osim model state files
kwargs['conversion'] = True            # Yes, perform 'Cvt#' process
kwargs['validation'] = True            # Yes, perform 'Vlt#' process
kwargs['speedy'] = True               # Do not reduce the checking notes to increase speed
kwargs['generate_pdf'] = True         # Do not generate validation pdf report
kwargs['add_ground_geom'] = True       # Add ground to the model
kwargs['treat_as_normal_path_point'] = False      # Using constraints to represent moving and conditional path points


############### 6D Neck model ################
osim_file = './models/osim/Neck6D/HYOID_1.2_ScaledStrenght_UpdatedInertia_rr.osim'
geometry_folder = './models/osim/Neck6D/Geometry'
output_folder = './myoconverter/tests/resource/joint_range_test/Neck6D'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)