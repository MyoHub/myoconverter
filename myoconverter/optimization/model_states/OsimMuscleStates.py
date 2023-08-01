#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb 20 21:43:21 2022

@author: hwang
"""

from myoconverter.optimization.utils.UtilsOpensim import getJointsControlledByMuscle, getCouplingJoints,\
        getAllIndependentCoordinates, getJointRanges_dict, getJointRanges_array, computeMomentArm,\
        getMuscleLengthList, getMuscleForceMaps, getMuscleProperties
from loguru import logger
import pickle
        

class OsimMuscleStates:
    """ A class to extract the muscle states (moment arms and force properites)
        of the given OpenSim model
     """
             
    def __init__(self, osim_model, muscle_list = None):
        """
        osim_model: loaded opensim model
        muscle_list: specified muscles whose states will be extracted. if None,
                     then all muscles inside the OsimModel will be calculated.
        """
        
        # initilize the input osim model as the global variables
        self.osim_model = osim_model
        if type(muscle_list) != list:
            self.muscle_list = [muscle_list]
        else:
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
        Extract the wrapping coordinates of the given muscle list. In addition,
        the coupling coordinates when calculating moment arms were extracted.
        
        For a given muscle list, outputs of this function is a dictionary:
            
            Keywords  ||  list of coordinates
            muscle 1: coordinate 1, coordinate 2
                      coordinate 3,
                      
            muscle 2: coordinate 1
                      coordinate 3, coordinate 4
                      coordinate 5, coordinate 6,
                      
            ...     
            
        """       

        logger.info("Extract the joints that muscle wrapped over from Osim model") 
        
        self.wrapping_coordinate = {}

        osim_muscles = self.osim_model.getMuscles()  # osim muscle class
            
        # go through a for loop of each muscle to extract the wrapped coordinates
        for i_muscle in self.muscle_list:
            
            # get wrapping coordiantes
            joints = getJointsControlledByMuscle(self.osim_model, osim_muscles.get(i_muscle))
            
            jointCP = getCouplingJoints(self.osim_model, osim_muscles.get(i_muscle), joints)
            
            # remove the duplicated coupling joints combinations
            wrap_list = [list(i) for i in {*[tuple(sorted(i)) for i in jointCP]}];
            
            # remove the sub coupling joints and only save the largest one.
            # Also make sure they always have the same order in every calculations.
            # A dirty fix right now, must have better ways to do this....
            i_wrap = 0
            while i_wrap < len(wrap_list):
                j_wrap = 0
                while j_wrap < len(wrap_list):
                    if i_wrap == j_wrap:
                        j_wrap = j_wrap + 1
                    else:
                        if all(elem in wrap_list[j_wrap] for elem in wrap_list[i_wrap]):
                            del wrap_list[i_wrap]
                            break
                        else:
                            j_wrap = j_wrap + 1
                else:
                    i_wrap = i_wrap + 1
                    
            self.wrapping_coordinate[i_muscle] = wrap_list
            
        return self.wrapping_coordinate
    
    # getting all coordinate ranges that wrapped by muscles
    def getCoordRanges(self):
        """
        Extract the motion ranges of given coordinates in an OpenSim model.
        
        Outputs of this function is a dictionary with the coordinate names as 
        keywords
        
               Keywords  ||   list
            
            coordinate 1:  [lower bound, upper bound]
            coordinate 2:  [lower bound, upper bound]
            coordinate 3:  [lower bound, upper bound]
            ...
        
        """

        logger.info("Extract joint motion range from Osim model")
        
        joint_names, DOF_Index = getAllIndependentCoordinates(self.osim_model)
        
        self.coordinate_range = getJointRanges_dict(self.osim_model, joint_names)
                    
        return self.coordinate_range

            
    def getMomentArms(self, save_path, speedy = False):
        """
        calcualte moment arms of a muscle at different coordinates it wrapped.

        Parameters
        ----------
        wrapping_coordinate : dictionary
            The dictionary that contains the muscles list and their
            wrapping coordinates. If not provide, will calculate from the above
            getWrapingCoord function.
            
        coordinate_range: dictionary
            The dictionary that contains the motion ranges of the
            joint lists that been wrapped by the muscles. if not provided,
            generate from the above getWrapingCoord function.

        save_path: string
            The path to save muscle-specific force map files, if not provided, not saving...
            
        speedy: boolean
            If True, select a lower number of particles, checking notes, iterations
            to speed up the optimization process.

        Returns
        -------
        self.moment_arms: dictionary
            The dictionary that contains the moment arms that were
            calculated from the opensim model with the given inputs
        """
        
        self.getWrapingCoords()  # generate wrapping coordiantes
        #  self.getCoordRanges()  # extract joint ranges

        logger.info("Calculate moment arms from Osim model")
            
        # self.moment_arms = {}  don't not export all the moment arm in one file anymore 
        for muscle in self.wrapping_coordinate.keys():   # run through all muscles

            logger.info(f"Muscle: {muscle}")
        
            osim_muscle  = self.osim_model.getMuscles().get(muscle)

            mus_para = {}
            mus_para['muscle_name'] = muscle
            muscle_moment_arms = []
            mus_para['wrapping_coordinates'] = self.wrapping_coordinate[muscle]

            motion_ranges = []
            moment_arms_joints = []
            evalN_list = []

            for joints in self.wrapping_coordinate[muscle]:  # run through all joints

                if speedy:  # speedy optimization with lower number of evaluation nodes
                
                    # setup the maximum mesh points depends on the number of 
                    # joint coupling together, otherwise it will take too long to
                    # compute them!
                    if len(joints) == 1:
                        evalN = 25
                    elif len(joints) == 2:
                        evalN = 7
                    elif len(joints) == 3:
                        evalN = 4
                    elif len(joints) == 4:
                        evalN = 3
                    else:
                        evalN = 2
                
                else:
                    # setup the maximum mesh points depends on the number of 
                    # joint coupling together, otherwise it will take too long to
                    # compute them!
                    if len(joints) == 1:
                        evalN = 25
                    elif len(joints) == 2:
                        evalN = 11
                    elif len(joints) == 3:
                        evalN = 7
                    elif len(joints) == 4:
                        evalN = 5
                    elif len(joints) == 5:
                        evalN = 4
                    else:
                        evalN = 3
            
                motion_range = []
                for joint in joints:  # extract the motion range
                    motion_range.append(getJointRanges_array(self.osim_model, joint))
                
                # calculate moment arms for this muscle and joints
                # and save it to a list
                moment_arms_joints.append(computeMomentArm(self.osim_model,\
                                                 osim_muscle, joints, motion_range,\
                                                 evalN))

                motion_ranges.append(motion_range)
                evalN_list.append(evalN)

            # save joint motion ranges and moment arm lists
            mus_para['osim_coordinate_ranges'] = motion_ranges
            mus_para['osim_ma_data'] = moment_arms_joints
            mus_para['evalN'] = evalN_list

            # save moment arm data if a saving path is provided
            if save_path:
                muscle_file = open(save_path + '/' + muscle + '.pkl', 'wb')
                pickle.dump(mus_para, muscle_file)
                muscle_file.close()

            # save the moment arms of the muscle into a dictionary, NO LONGER NEEDED
            # self.moment_arms[muscle]= muscle_moment_arms
            
        # return self.moment_arms
    
    def getMuscleForceMaps(self, save_path, speedy = False):
    
        """get the muscle force maps from OpenSim models. 
        
        Parameters
        ----------
        
        save_path: string
            The path to save muscle-specific force map files, if not provided, not saving...
            
        speedy: boolean
            If True, select a lower number of particles, checking notes, iterations
            to speed up the optimization process.

        Returns (NONE)
        -------
        self.force_maps: dictionary
            The dictionary that contains the muscle force maps that were
            calculated from the opensim model with the given inputs
        """
        # TODO: set condition to avoid run this again.
        self.getWrapingCoords()

        logger.info("Calculate muscle force curves from Osim model")
            
        # force_maps = {}
        for muscle in self.wrapping_coordinate.keys():   # run through each muscle

            logger.info(f"Muscle: {muscle}")
        
            osim_muscle  = self.osim_model.getMuscles().get(muscle)
            
            # get a list of muscle lengths from minimal to maximum and the corresponding
            # joint angle list
            force_map = {}
            
            # extract the property parameters from the OpenSim models
            force_map['mtu_par_set'] = getMuscleProperties(osim_muscle)
            
            motion_range = []
            joints_uniq = []
            for joints in self.wrapping_coordinate[muscle]:  # run through all joints           
                for joint in joints:  # extract the motion range
                    if joint not in joints_uniq:  # only go through the unique joint coordinates
                        joints_uniq.append(joint)
                        motion_range.append(getJointRanges_array(self.osim_model, joint))
        
            mtu_len_set, jit_list_set = getMuscleLengthList(self.osim_model,\
                                                            osim_muscle, joints_uniq,\
                                                            motion_range, speedy)
            
            force_map['muscle_name'] = muscle
            force_map['mtu_length_osim'] = mtu_len_set
            force_map['jit_list_set'] = jit_list_set
            force_map['jit_uniq'] = joints_uniq
            force_map['jit_uniq_ranges'] = motion_range
                
            # set the muscle activation shift from 0 to 1
            # only optimize when activation equal to 1
            act_list = [1] # np.linspace(0, 1, 5)
            force_map['act_list'] = act_list
                
            # get muscle tendon force,  and active/passive forces
            mtu_force_length, act_force_length, pas_force_length = \
                getMuscleForceMaps(self.osim_model, osim_muscle, joints_uniq, jit_list_set, act_list)
                
            force_map['mtu_force_osim'] = mtu_force_length
            force_map['act_force_osim'] = act_force_length
            force_map['pas_force_osim'] = pas_force_length
                
            if save_path:
                muscle_mf_file = open(save_path + '/' + muscle + '.pkl', 'wb')
                pickle.dump(force_map, muscle_mf_file)
                muscle_mf_file.close()
        
        # return force_maps
    
