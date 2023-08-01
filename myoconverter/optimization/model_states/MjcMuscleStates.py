#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 20 21:43:21 2022

@author: hwang
"""
import sys
sys.path.append('../utils')

import numpy as np
from loguru import logger
from myoconverter.optimization.utils.UtilsMujoco import computeMomentArmMusclesJoints, getMuscleForceLengthCurvesSim
        

class MjcMuscleStates:
    """ A class to extract the muscle states (moment arms and force properites)
        of the given Mujoco model
     """
             
    def __init__(self, mjc_model, wrapping_coordinate, muscle_list = None):
        """
        mjc_model: loaded mujoco model
        wrapping_coordinate: the wrapping coordinates from OpenSim model is 
                             needed here. Assume it will change for mjc model
                             due to the direct geometry translation.
        muscle_list: specified muscles whose states will be extracted. if None,
                     then all muscles inside the mjc model will be calculated.
        """
        
        # initilize the input osim model and muscle list as the global variables
        self.mjc_model = mjc_model
        self.wrapping_coordinate = wrapping_coordinate
        self.muscle_list = muscle_list
        
        # dictionary for storing calculated moment arms and force properties
        self.moment_arms = dict()
        self.force_properties = dict()
        
        
    def reset(self):
        # reset all calculated variables
        self.moment_arms = dict()
        self.force_properties = dict()
        
        
    # get the wrapping coordinate of the given muscle list
    def getWrapingCoords(self):
        """
        This information will directly take the result from Osim model, 
        from given input. 
        
        In mjc models, muscles are treated as one type of actuators (type 3), 
        there are several other type of actuators. Here we only look at the 
        muscle actuators.
        
        For a given muscle list, outputs of this function is a dictionary:
            
            Keywords  ||  list of coordinates
            muscle 1: coordinate 1, coordinate 2
                      coordinate 3,
                      
            muscle 2: coordinate 1
                      coordinate 3, coordinate 4
                      coordinate 5, coordinate 6,
                      
            ...     
            
        """ 


        # if muscle list is not provided, then explore all muscles that included
        # in the MuJoCo model.
        if not self.muscle_list:
            # update the global muscle list variable with all muscle names inside
            # the mjc model, only the type of muscle actuator
            self.muscle_list = [list(self.mjc_model.actuator_names)[i] \
                                for i, val in enumerate(self.mjc_model.actuator_trntype)\
                                if val == 3]
            
        else:
            # if the muscle list is provided, check if they are included in the mjc model
            mjc_muscle_list = list(self.mjc_model.actuator_names)
            
            for muscle in self.muscle_list:
                if muscle not in mjc_muscle_list:
                    raise('The provided muscle ' + muscle + ' is not included in the mjc model')

        # check if the provided or extracted mjc muscle list be covered by the
        # wrapping_coordinates that extracted from the opensim model
        for muscle in self.muscle_list:
            if muscle not in self.wrapping_coordinate.keys():
                raise('The muscle ' + muscle + ' is not included in the provided wrapping_coordinate')
        
        return self.wrapping_coordinate
    
    # getting all coordinate ranges that wrapped by muscles
    def getCoordRanges(self):
        """
        Extract the motion ranges of given coordinates in an mjc model.
        
        Outputs of this function is a dictionary with the coordinate names as 
        keywords
        
               Keywords  ||   list
            
            coordinate 1:  [lower bound, upper bound]
            coordinate 2:  [lower bound, upper bound]
            coordinate 3:  [lower bound, upper bound]
            ...
        
        """
        
        logger.info("Extract joint motion range from Mjc model")

        self.coordinate_range = {}
        
        jnt_names = self.mjc_model.joint_names
        jnt_ranges = self.mjc_model.jnt_range
            
        # go through a for loop of each joint to to formulate a dictionary
        self.coordinate_range = {jnt_names[i]:jnt_range for i, jnt_range in enumerate(jnt_ranges)}
                    
        return self.coordinate_range
    
    def getMomentArms(self, wrapping_coordinate = None, coordinate_range = None, evalN = 7):
        """
        calcualte moment arms of a muscle at different coordinates it wrapped.

        Parameters
        ----------
        wrapping_coordinate : dictionary
            DESCRIPTION: the dictionary that contains the muscles list and their
            wrapping coordinates. If not provide, will calculate from the above
            getWrapingCoord function.
            
        coordinate_range: dictionary
            DESCRIPTION: the dictionary that contains the motion ranges of the
            joint lists that been wrapped by the muscles. if not provided,
            generate from the above getWrapingCoord function.
            
        evalN: integer
        DESCRIPTION: Number of nodes to evalute in between the coordinate range, default value 7.

        Returns
        -------
        self.moment_arms: dictionary
            DESCRIPTION: the dictionary that contains the moment arms that were
            calculated from the opensim model with the given inputs
        """
        
        logger.info("Calculate moment arms from Mjc model")

        if wrapping_coordinate == None:  # if not provided, generate it
            wrapping_coordinate = self.getWrapingCoords()
        
        if coordinate_range == None:  # if not provided, generate it
            coordinate_range = self.getCoordRanges()
            
        self.moment_arms = {}
        for muscle in wrapping_coordinate.keys():   # run through all muscles

            logger.info(f"Muscle: {muscle}")
        
            muscle_moment_arms = []
            for joints in wrapping_coordinate[muscle]:  # run through all joints
                
                motion_range = []
                for joint in joints:  # extract the motion range
                
                    # setup the maximum mesh points depends on the number of 
                    # joint coupling together, otherwise it will take too long to
                    # compute them!
                    if len(joints) == 1:
                        evalN = 25
                    elif len(joints) == 2:
                        evalN = 11
                    elif len(joints) == 3:
                        evalN = 7
                    else:
                        evalN = 5

                    motion_range.append(coordinate_range[joint])
                
                # calculate moment arms for this muscle and joints
                # and save it to a list
                
                muscle_moment_arms.append(computeMomentArmMusclesJoints(self.mjc_model,\
                                                                 muscle, joints,\
                                                                 motion_range, evalN))
                    
            # save the moment arms of the muscle into a dictionary
            self.moment_arms[muscle]= muscle_moment_arms
            
        return self.moment_arms
    
    def getMuscleForceMaps(self, osim_force_maps, wrapping_coordinate = None, coordinate_range = None, evalN = 11):
        """
        get the muscle force maps from MuJoCo models. Strongly suggest to run 
        this step after optimizing the moment arms. This force length maps are 
        using the same joint angle vectors that been used in OpenSim model when
        force maps were extracted. If there are large moment arm differences, 
        the muscle lengths will be very different, then comparison between the
        OpenSim and MuJoCo muscle force maps are not vaildate any more.
        
        Parameters
        ----------
        osim_force_maps: dictionary
            DESCRIPTION: the force maps exracted from OpenSim model.
            
        wrapping_coordinate : dictionary
            DESCRIPTION: the dictionary that contains the muscles list and their
            wrapping coordinates. If not provide, will calculate from the above
            getWrapingCoord function.
            
        coordinate_range: dictionary
            DESCRIPTION: the dictionary that contains the motion ranges of the
            joint lists that been wrapped by the muscles. if not provided,
            generate from the above getWrapingCoord function.
            
        evalN: integer
        DESCRIPTION: Number of nodes to evalute in between the coordinate range, default value 11.

        Returns
        -------
        self.force_maps: dictionary
            DESCRIPTION: the dictionary that contains the muscle force maps that were
            calculated from the opensim model with the given inputs
        """

        logger.info("Calculate muscle force curves from Mjc model")
    
        if wrapping_coordinate == None:  # if not provided, generate it
            wrapping_coordinate = self.getWrapingCoords()
        
        if coordinate_range == None:  # if not provided, generate it
            coordinate_range = self.getCoordRanges()
            
        self.force_maps = {}
        for muscle in osim_force_maps.keys():   # run through each muscle

            logger.info(f"Muscle: {muscle}")
        
            # get a list of muscle lengths from minimal to maximum and the corresponding
            # joint angle list
            force_map = {}
            
            jit_list_set = osim_force_maps[muscle]['jit_list_set']
            
            motion_range = []
            joints_uniq = []
            for joints in wrapping_coordinate[muscle]:  # run through all joints
                for joint in joints:  # extract the motion range
                    if joint not in joints_uniq:  # only go through the unique joint coordinates
                        joints_uniq.append(joint)
                        motion_range.append(coordinate_range[joint])

            # set the muscle activation shift from 0 to 1
            act_list = [1]
                
            # get muscle tendon force,  and active/passive forces
            mtu_force_length, mtu_len_set = \
                getMuscleForceLengthCurvesSim(self.mjc_model, muscle, joints_uniq, 
                                   jit_list_set, act_list)
                

            force_map['mtu_len_set'] = mtu_len_set
            force_map['mtu_force_length'] = mtu_force_length
            
            self.force_maps[muscle] = force_map
        
        return self.force_maps
    
