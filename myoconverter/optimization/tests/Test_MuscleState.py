#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OsimMuscleStates Testing

Created on Tue Nov  1 22:57:35 2022

@author: hwang
"""

from myoconverter.optimization.model_states.OsimMuscleStates import OsimMuscleStates
import opensim

from myoconverter.optimization.model_states.MjcMuscleStates import MjcMuscleStates
import mujoco_py

osim_model_path = '../../../models/osim/mobl_arms/MOBL_ARMS_fixed_41.osim'

# load osim model
osim_model = opensim.Model(osim_model_path)

# get muscle states
osimMusSta = OsimMuscleStates(osim_model)

# get the coordinates that are wrapped by these muscles
wrapping_coordinate = osimMusSta.getWrapingCoords()

# get the coordinate ranges
osim_coordinate_range = osimMusSta.getCoordRanges()

# calculate moment arms of all wrapping coordinate, a subset can be selected to
# speed up this test
osim_moment_arms = osimMusSta.getMomentArms(wrapping_coordinate = wrapping_coordinate,\
                                       coordinate_range = osim_coordinate_range, evalN = 3)
    
      
# plotting osim coordinate ranges and moment arm maps as illustration, 
# TO BE ADDED (need discuss how to show)




## now testing the mjc muscle state class

mjc_model_path = '../../../models/converted/mobl_arms/MOBL_ARMS_fixed_41.xml'

# load mjc model
mjc_model = mujoco_py.load_model_from_path(mjc_model_path)

# initalized the class
mjcMusSta = MjcMuscleStates(mjc_model, wrapping_coordinate)

# calculate coordinate ranges
mjc_coordinate_range = mjcMusSta.getCoordRanges()

# calculate moment arms
mjc_moment_arms = mjcMusSta.getMomentArms(wrapping_coordinate = wrapping_coordinate,\
                                          coordinate_range = mjc_coordinate_range, evalN = 3)
    

# plotting mjc coordinate ranges and moment arm maps as illustration, 
# TO BE ADDED (need discuss how to show)




                                       



