#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 28 12:54:25 2021

@author: hwang
"""

import mujoco
from myoconverter.optimization.utils.UtilsMujoco import updateWrapSites, computeMomentArmMuscleJoints,\
    getMuscleForceLengthCurvesSim, updateMuscleForceProperties

import numpy as np
from numpy import pi, sqrt

from loguru import logger
import multiprocessing as mp

# @logger.catch
def objMAMuscle(x, side_id, wrap_type, wrap_id, pos_wrap, size_wrap, rotation_wrap,\
                osim_ma, mjc_model_path, muscle, joints, joint_ranges, evalN):
    """ this objective function calculate the difference of moment arms between opensim model and mujoco model.
	
	Inputs:

        x: vector
            optimization vectors
        side_id: int
            the id list of the optimization sides
        wrap_type: string
            the type defined in O2MStep2
        wrap_id: int
            wrapping object id
        pos_wrap: vector
            the position of the wrapping object
        size_wrap: vector
            the size of the wrapping object
        rotation_wrap: vector
            the rotation vector of the wrapping object
        osim_ma: vector
            the moment arm vector of the osim model
        mjc_model: mujoco model
            the targeting mujoco model
        muscle: string
            muscle name
        joints: list of string
            the coordinates that the moment arms are caclulated
        angle_ranges: list
            the motion ranges of the mentioned coordinates
        evalN: int
            the number of evaluation points for the moment arm calculation

	Outputs:
		rms_ma: root mean square of the moment arm difference.
        rms_x: root mean square of the optimizing parameters. 
	"""

    mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)

    if type(joints) != list:  # check if the joints are in a list or not
        joints = [joints]
            
    rms_ma = 0   # initialize the rms_ma
   
   	# update mjc model with new attaching_list and wrapping_list
    mjcModel = updateWrapSites(mjc_model, wrap_type, wrap_id, pos_wrap, size_wrap, rotation_wrap, side_id, x)
   
   	# calculate moment arm
    mjc_ma = computeMomentArmMuscleJoints(mjcModel, muscle, joints, joint_ranges, evalN)
        
    # check the list length
    if len(osim_ma) != len(mjc_ma):
        logger.debug("osim and mjc models have different number wrapping coordinates of this wrapping object")
        raise('osim and mjc models have different number wrapping coordinates of this wrapping object')
        
    rms_ma = 0
       
    if len(np.array(osim_ma).flatten()) != len(np.array(mjc_ma).flatten()):
        logger.debug("osim and mjc models have different evalN numbers")
        raise('osim and mjc models have different evalN numbers\n')
       
    # calculate moment arm differences
    # osim and mjc has opposite sign in MA, so 'add' is used ...
    # also change the value from m to cm to have better understanding...
    rms_ma = np.sqrt(np.sum((np.array(osim_ma).flatten() + np.array(mjc_ma).flatten())**2)/len(np.array(mjc_ma).flatten())) 
    
    return rms_ma

@logger.catch
def getMomentArmDiff(mjc_model, muscle, joints, jnt_motion_range, osim_ma_joints, evalN):
    '''
    Check if the moment arm differences between the osim and mjc models are beyond
    thresholds

    Only works for one muscle at multiple joints.
    '''
    
    err_ind = []
    mjc_ma_joints = []
          
    # joints should always be a list, even only one joint exist
    if type(joints) != list:
        joints = [joints]
    
    # get moment arms from the mjc model
    mjc_ma_joints = np.squeeze(computeMomentArmMuscleJoints(mjc_model,\
                                                        muscle, joints,\
                                                        jnt_motion_range, evalN))
    
    # check the list length
    if len(osim_ma_joints) != len(mjc_ma_joints):
        logger.debug("osim and mjc models have different number of muscles of this wrapping object")
        raise RuntimeError('osim and mjc models have different number of muscles of this wrapping object')
        
    obj_org = 0
    # calculate differences  
    osim_ma_joints_array = (np.array(osim_ma_joints)).flatten()
    mjc_ma__joints_array = (np.array(mjc_ma_joints)).flatten()
    
    obj_org = obj_org + np.sqrt(np.sum(( osim_ma_joints_array + mjc_ma__joints_array )**2)/len(osim_ma_joints_array))

    # absolute and relative errors
    abs_err = abs(osim_ma_joints_array + mjc_ma__joints_array)
    
    osim_ma_joints_array_nonzeros = np.maximum(abs(osim_ma_joints_array), 1e-3)
    
    rel_err = abs((osim_ma_joints_array + mjc_ma__joints_array)/(osim_ma_joints_array_nonzeros))
    
    # check if absolulte error larger than 0.001 and relative error larger than 5%
    err_ind = (list(set(np.where(abs_err > 0.001)[0]) & set(np.where(rel_err > 0.05)[0])))
    
    return err_ind, obj_org

# optimize wrapping side using the self-developed PSO Optimizer
# @logger.catch
def maOptPSO_cust(mjc_model_path, muscle, joints, joint_ranges, side_id, wrap_type, wrap_id, pos_wrap, size_wrap,\
               rotation_wrap, osim_ma, evalN, optParam_lb, optParam_ub, cost_org, speedy = False):
    """
    This is a self-defined particle swarm optimizer (PSO) for the moment arm optimization. The purpurse of define
    it here, instead of using other packages, is to gain custmizied feature control. 
    This is a temporary solution, eventually, these features should be added to the exist PSO python packages and
    use them as the optimizor

    INPUTS:
        mjcModel: mujoco model
        muscle: string
            muscle name
        joints: list
            a list of coordinate names for the optimization
        joint_ranges: list
            a list of vectors that defines the motion ranges of the above coordinates
        side_id: int
            the id list of the optimization sides
        wrap_type: string
            the type defined in O2MStep2
        wrap_id: int
            wrapping object id
        pos_wrap: vector
            the position of the wrapping object
        size_wrap: vector
            the size of the wrapping object
        rotation_wrap: vector
            the rotation vector of the wrapping object
        osim_ma: vector
            the moment arm vector of the osim model
        evalN: int
            the number of evaluation points for moment arm opt
        opt_param_lb: array
            the lower bound of the optimized parameters
        opt_param_ub: array
            the upper bound of the optimized parameters
        cost_org: double
            the cost function value before the optimization
        speedy: boolean
            If true, reduce the particle size and iteration number

    OUTPUTS:
        res_side: optimization side locations
        mjcModel: updated mujoco models
    """
    
    # make sure the boundaries are array
    if type(optParam_lb) != np.ndarray:
        optParam_lb = np.array(optParam_lb)
        
    if type(optParam_ub) != np.ndarray:
        optParam_ub = np.array(optParam_ub)

    # PSO options
    c1  = 0.3   # inherit rate from the best of the current particle itself
    c2 = 0.3  # inherit rate from the global best particle
    w = 0.4   # inherit rate from the previous velocity
    
    dimensions = len(optParam_lb)  # optimizing parameter demensions
    
    if speedy:
        n_particles = 10
        iteration_max = 25
        break_threshold = 0.25  # when more than #% particles have the same values, then stop the optimization,
    else:
        n_particles = 10*dimensions  # particle number is set to be 10 times the parameter demensions
        iteration_max = 50
        break_threshold = 0.5
        
    
    # initilize the optimization variables randomly inside the bounds
    x = optParam_lb + np.random.uniform(low = 0, high = 1,\
             size = (n_particles, dimensions))*(optParam_ub - optParam_lb)
        
    # initilize the velocities randomly inside the bounds
    v = (optParam_lb - optParam_ub) + np.random.uniform(low = 0, high = 1,\
             size = (n_particles, dimensions))*(optParam_ub - optParam_lb)*2
        
    # the best cost function for each particle is initilized as 0
    obj_b = np.zeros(n_particles)
    
    p = x  # assign the particle values

    obj_g = []  # initilize an empty global cost function value
    obj_g_old = []  # old cost function value
    obj_g_iter = 0 # number of iterations that contain the same obj_g

    itera = 0  # interation starts
    
    similar_particles = 0  # number of similar particles, this will be used as an cretiria to stop the optimization

    with mp.Pool() as pool:
        
        while itera < iteration_max:   # define the maximum iteration value
            
            # Apply parallel computing using multiprocessing
            # Right now, the mujoco sim cannot be pickled and transfer to objective function
            # To solve this, mujoco file name need to transfer to objective function and load over there.

            # prepare function inputs
            x_input = []
            for ix in x:
                x_input.append((ix, side_id, wrap_type, \
                                    wrap_id, pos_wrap, size_wrap,\
                                    rotation_wrap, osim_ma, mjc_model_path,\
                                    muscle, joints, joint_ranges, evalN))

            obj_list = pool.starmap(objMAMuscle, x_input)
            
            # update the local and global optimal
            if itera == 0:
                obj_b = obj_list
                obj_g = min(obj_list)
                obj_g_old = obj_g
                g = x[obj_list.index(obj_g)]
            else:
                for iobj, obj in enumerate(obj_list):
                    if obj < obj_b[iobj]:
                        obj_b[iobj] = obj
                        p[iobj] = x[iobj]

                    if obj < obj_g:
                        obj_g = obj
                        g = x[iobj]

            logger.info(f"        PSO iteration: {itera} ; {round(similar_particles*100/n_particles)} percentage similarities; Best obj: {np.round(obj_g, 5)}")
                    
            # two random values to increase intersection between particles
            r1 = np.random.rand(1)
            r2 = np.random.rand(1)
            
            v = w*v + c1*r1*(p - x) + c2*r2*(g - x)  # calculate velocities
            
            x = x + v  # change to next iteration positions
            
            # make sure they are within boundaries
            for i, ix in enumerate(x):
                x[i] = np.minimum(np.maximum(ix, optParam_lb), optParam_ub)
                
            itera = itera + 1 # increase the iteration number
            
            similar_particles = len(np.where((obj_b - min(obj_b))/min(obj_b) < 0.01)[0])
            
            # break if certain percentage of particles have similar objective values
            if similar_particles > break_threshold*n_particles:  
                logger.info("        Break the optimization, since certain number of similar particles reached")
                break

            # break if the obj_g is the same value for more than 10 iterations
            if obj_g == obj_g_old:
                obj_g_iter = obj_g_iter + 1
            else:
                obj_g_iter = 0
                
            obj_g_old = obj_g

            if obj_g_iter > 10:
                logger.info("        Break the optimization, since global obj maintained the same value for certain iterations")
                break

    mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)
    # update mjc model with new attaching_list and wrapping_list
    mjc_model = updateWrapSites(mjc_model, wrap_type, wrap_id, pos_wrap, size_wrap, rotation_wrap, side_id, g)
    
    # save optimized results    
    res_site = {"wrap_type": wrap_type, "cost_org": cost_org, "cost_opt": obj_g, "res_opt": g}
        
    return res_site, mjc_model
    
