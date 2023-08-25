#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 28 12:54:25 2021

@author: hwang
"""

import mujoco
from myoconverter.optimization.utils.UtilsMujoco import getMuscleForceLengthCurvesSim, updateMuscleForceProperties

import numpy as np
from numpy import pi, sqrt

from loguru import logger
import multiprocessing as mp

@logger.catch
def objFMMuscle(x, osim_fm, mjc_model_path, muscle, joints, jnt_arr, act_arr):
    """
    Calculate the muscle force differences between osim and mjc models.
    
    INPUTS:
        x: vector
            optimizing parameters
        osim_fm: vector/mat
            muscle force vector/matrix of a given muscle
        mjcModel: mujoco model
            mujoco model
        muscle: string
            muscle name
        joints: list of string
            a list of unique coordinate that affecting the muscle length
        jnt_arr: list
            a list of joint angle values for the above unique coordinates
        act_arr: list
            a list of muscle activation values

    OUTPUTS:
        rms_fm: double
            the RMS value of muscle force map differences
    """

    mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)

    mjc_model = updateMuscleForceProperties(mjc_model, muscle, x)
    
    mjc_fm, length_mtu = getMuscleForceLengthCurvesSim(mjc_model, muscle, joints, jnt_arr, act_arr)
    
    # calculate moment arm differences, osim and mjc has opposite sign in MA
    return np.sqrt(np.sum((np.array(osim_fm).flatten() + np.array(mjc_fm).flatten())**2)/len(np.array(mjc_fm).flatten())) 

@logger.catch
def getMuscleForceDiff(mjc_model, muscles, joints, jnt_arr, act_arr, osim_fp_muscle_joints):
    '''
    Check if the muscle force differences between the osim and mjc models are beyond
    thresholds
    '''
    err_ind = []
    
    force_mtu, length_mtu = getMuscleForceLengthCurvesSim(mjc_model, muscles, joints, jnt_arr, act_arr)
    
    # change ndarray to array
    osim_fp_muscles_joints_array = np.array(osim_fp_muscle_joints).flatten()
    mjc_fp_muscles_joints_array = np.array(force_mtu).flatten()
    
    if len(osim_fp_muscles_joints_array) != len(mjc_fp_muscles_joints_array):
        logger.debug("osim and mjc models have different moment arm sizes")
        raise('osim and mjc models have different moment arm sizes')
    
    obj_org = np.sqrt(np.sum((osim_fp_muscles_joints_array + mjc_fp_muscles_joints_array)**2)/len(mjc_fp_muscles_joints_array))
        
    # absolute and relative errors
    abs_err = abs(osim_fp_muscles_joints_array + mjc_fp_muscles_joints_array)
    
    osim_fp_muscles_joints_array_nonzeros = np.maximum(abs(osim_fp_muscles_joints_array), 1e-3)
    
    rel_err = abs((osim_fp_muscles_joints_array + mjc_fp_muscles_joints_array)/(osim_fp_muscles_joints_array_nonzeros))
    
    # check if absolulte error larger than 0.001 and relative error larger than 5%
    err_ind.append(list(set(np.where(abs_err > 0.001)[0]) & set(np.where(rel_err > 0.05)[0])))
    
    return err_ind, length_mtu, obj_org


# optimize muscle forces using the self-developed PSO Optimizer
@logger.catch
def fmOptPSO_cust(mjc_model_path, muscle, joints, jnt_arr, act_arr,\
                      osim_fm, optParam_lb, optParam_ub, cost_org, speedy = False):
    
    # make sure the boundaries are array
    if type(optParam_lb) != np.ndarray:
        optParam_lb = np.array(optParam_lb)
        
    if type(optParam_ub) != np.ndarray:
        optParam_ub = np.array(optParam_ub)

    # PSO options
    c1  = 0.3   # inherit rate from the best of the current particle itself
    c2 = 0.25  # inherit rate from the global best particle
    w = 0.25   # inherit rate from the previous velocity
    
    dimensions = len(optParam_lb)  # optimizing parameter demensions
    
    if speedy:
        n_particles = 5*dimensions
        iteration_max = 25
        break_threshold = 0.25  # when more than #% particles have the same values, then stop the optimization,
    else:
        n_particles = 20*dimensions  # particle number is set to be 20 times the parameter demensions
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
        
        while itera < iteration_max:

            # Apply parallel computing using multiprocessing
            # Right now, the mujoco sim cannot be pickled and transfer to objective function
            # To solve this, mujoco file name need to transfer to objective function and load over there.
            # This will change the entire strucutre of the optimization process, not a day of work. 

            # prepare function inputs
            x_input = []
            for ix in x:
                x_input.append((ix, osim_fm, mjc_model_path, muscle, joints, jnt_arr, act_arr))

            obj_list = pool.starmap(objFMMuscle, x_input)
            
            # update the local and global optimal
            if itera == 0:
                obj_b = obj_list
                obj_g = min(obj_list)
                g = x[obj_list.index(obj_g)]
            else:
                for iobj, obj in enumerate(obj_list):
                    if obj < obj_b[iobj]:
                        obj_b[iobj] = obj
                        p[iobj] = x[iobj]

                    if obj < obj_g:
                        obj_g = obj
                        g = x[iobj]
        
            # two random values to increase intersection between particles
            r1 = np.random.rand(1)
            r2 = np.random.rand(1)
            
            v = w*v + c1*r1*(p - x) + c2*r2*(g - x)  # calculate velocities
            
            x = x + v  # change to next iteration positions
            
            # make sure they are within boundaries
            for i, ix in enumerate(x):
                x[i] = np.minimum(np.maximum(ix, optParam_lb), optParam_ub)

            logger.info(f"        PSO iteration: {itera} ; {round(similar_particles*100/n_particles)} percentage similarities; Best obj: {np.round(obj_g, 5)}")
                
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
    # update mjc model with new muscle parameters
    mjc_model = updateMuscleForceProperties(mjc_model, muscle, g)
    
    # save optimized results    
    opt_results = {"cost_org": cost_org, "cost_opt": obj_g, "res_opt": g}
        
    return opt_results, mjc_model
    


        
            
        
        
