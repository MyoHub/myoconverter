#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  1 13:51:09 2022

@author: hwang
"""

from myoconverter.optimization.model_states.OsimMuscleStates import OsimMuscleStates
from myoconverter.optimization.utils.UtilsForceOpt import getMuscleForceDiff, fmOptPSO_cust
from myoconverter.optimization.utils.UtilsMujoco import getMuscleForceLengthCurvesSim, getCoordinateRange_mjc
import mujoco
import opensim
import os
import pickle
import glob
import numpy as np
import matplotlib.pyplot as plt 
from loguru import logger

class MuscleForceOpt:
    """
    Class to optimize muscle force properties of MJC model
    """
    
    def __init__(self, mjc_model_path, osim_model_path, saving_path, \
                 muscle_list = None, osim_data_overwrite = False, speedy = False):
        """
        Parameters
        ----------
        mjc_model_path : string
            The model path of mjc model
            
        osim_model_path : string
            The model path of osim model
            
        saving_path : string
            The path to save moment arm results.
            
        muscle_list : list of string, optional
            List of muscle names whose moment arms will be optimized
            
        osim_data_overwrite : boolean, optional
            If True, overwrite osim state data

        speedy : boolean, optional
            If True, select a lower number of particles, checking notes, iterations
            to speed up the optimization process.
            
        Returns
        -------
        None.
        """
        
        # load both mujoco and opensim models
        self.mjc_model_path = mjc_model_path
        
        self.mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)
        self.osim_model = opensim.Model(osim_model_path)
        
        # if muscle list is not provided, then explore all muscles that included
        # in the OpenSim model.
        if not muscle_list:

            osim_muscles = self.osim_model.getMuscles()  # osim muscle class
            
            # update the global muscle list variable with all muscle names inside
            # the OpenSim model
            self.muscle_list = []
                        
            for i in range(osim_muscles.getSize()):
                self.muscle_list.append(osim_muscles.get(i).getName())

        else:
            self.muscle_list = muscle_list  # provided muscle list, could be None.

        # saving path has to be provided and valid
        try:
            os.makedirs(saving_path, exist_ok = True)
        except OSError as exc:
            raise ValueError("Please provide a valid saving path") from exc

        self.save_path = saving_path

        # Check if the osim_data needs to be over write or not.
        # If so, generate them using the modelState/OsimMuscleStates class
        # Another case to regenerate is that there is no data file saved 
        # in the saving path of the provided muscle

        # get mjc joint ranges
        ang_ranges_mjc, free_jnt_id_mjc = getCoordinateRange_mjc(self.mjc_model)

        if osim_data_overwrite:

            logger.info("Overwrite command confirmed")
            logger.info("Generating force map data from OsimMuscleStates, may take a while")

            # This will take a while, previous generated MF data files of selected muscles will be overwrited
            osim_mus_sta = OsimMuscleStates(self.osim_model, self.muscle_list)

            # regenerate moment arms
            osim_mus_sta.getMuscleForceMaps(save_path = self.save_path, speedy = speedy)

            for mus_name in self.muscle_list:
                # add mujoco unique joint arr information, in case the joint range are different.
                # load saved muscle force data file
                with open(self.save_path + '/' + mus_name + '.pkl', "rb") as muscle_file:
                    muscle_para_mf = pickle.load(muscle_file)
                
                # first deep copy the jit list set
                muscle_para_mf['mjc_jit_list_set'] = muscle_para_mf['jit_list_set'].copy()

                # then test if the motion range is the same for each unique joint
                for i_unique_jit, unique_jit in enumerate(muscle_para_mf['jit_uniq']):

                    osim_range = (muscle_para_mf['jit_uniq_ranges'][i_unique_jit][1] - \
                                         muscle_para_mf['jit_uniq_ranges'][i_unique_jit][0])
                    mjc_range = (ang_ranges_mjc[unique_jit][1] - \
                                         ang_ranges_mjc[unique_jit][0])

                    # count for decimal round, small difference does not count. Just check range difference here
                    if not (np.abs(osim_range - mjc_range) < 0.005):

                        # change all the joint list set of cooresponding joint
                        for i_set in range(len(muscle_para_mf['mjc_jit_list_set'])):

                            jit_set_list = list(muscle_para_mf['mjc_jit_list_set'][i_set])

                            jit_set_list[i_unique_jit] =\
                                  (muscle_para_mf['jit_list_set'][i_set][i_unique_jit] - \
                                      muscle_para_mf['jit_uniq_ranges'][i_unique_jit][0]) \
                                      *mjc_range/osim_range + ang_ranges_mjc[unique_jit][0]
                            
                            muscle_para_mf['mjc_jit_list_set'][i_set] = tuple(jit_set_list)

                # save moment arm data if a saving path is provided
                with open(self.save_path + '/' + mus_name + '.pkl', 'wb') as muscle_saving:
                    pickle.dump(muscle_para_mf, muscle_saving)
                    muscle_saving.close()

        else:

            logger.info("Overwrite not required")
            logger.info("Checking if the muscle data file exist")

            for mus_name in self.muscle_list:
                if not os.path.isfile(self.save_path + '/' + mus_name + '.pkl'):
                    logger.info(f"Muscle: {mus_name} data file does not exist, regenerating")

                    osim_mus_sta = OsimMuscleStates(self.osim_model, mus_name)

                    # regenerate moment arms
                    osim_mus_sta.getMuscleForceMaps(save_path = self.save_path, speedy = speedy)

                    # add mujoco unique joint arr information, in case the joint range are different.
                    # load saved muscle force data file
                    with open(self.save_path + '/' + mus_name + '.pkl', "rb") as muscle_file:
                        muscle_para_mf = pickle.load(muscle_file)
                    
                    # first deep copy the jit list set
                    muscle_para_mf['mjc_jit_list_set'] = muscle_para_mf['jit_list_set'].copy()

                    # then test if the motion range is the same for each unique joint
                    for i_unique_jit, unique_jit in enumerate(muscle_para_mf['jit_uniq']):

                        osim_range = (muscle_para_mf['jit_uniq_ranges'][i_unique_jit][1] - \
                                            muscle_para_mf['jit_uniq_ranges'][i_unique_jit][0])
                        mjc_range = (ang_ranges_mjc[unique_jit][1] - \
                                            ang_ranges_mjc[unique_jit][0])

                        # count for decimal round, small difference does not count. Just check range difference here
                        if not (np.abs(osim_range - mjc_range) < 0.005):

                            # change all the joint list set of cooresponding joint
                            for i_set in range(len(muscle_para_mf['mjc_jit_list_set'])):

                                jit_set_list = list(muscle_para_mf['mjc_jit_list_set'][i_set])

                                jit_set_list[i_unique_jit] =\
                                  (muscle_para_mf['jit_list_set'][i_set][i_unique_jit] - \
                                      muscle_para_mf['jit_uniq_ranges'][i_unique_jit][0]) \
                                      *mjc_range/osim_range + ang_ranges_mjc[unique_jit][0]
                            
                                muscle_para_mf['mjc_jit_list_set'][i_set] = tuple(jit_set_list)

                    # save moment arm data if a saving path is provided
                    with open(self.save_path + '/' + mus_name + '.pkl', 'wb') as muscle_saving:
                        pickle.dump(muscle_para_mf, muscle_saving)
                        muscle_saving.close()

                else:
                    logger.info(f"Muscle: {mus_name} data file exist, will reuse it in the optimization")

        logger.info('Finished cvt3 initialize ...')

    def optMuscleForce(self):

        # Right now only optimize muscle force length relationship when activation
        # equal to 1
        
        # Set all fmax and scale factor as 1, to avoid very large muscle forces
        for ig in range(len(self.mjc_model.actuator_gainprm)):
            self.mjc_model.actuator_gainprm[ig][2] = 1
            self.mjc_model.actuator_gainprm[ig][3] = 1
            
        # start optimization
        logger.info('Running MF optimization with the given muscle list one by one')

        for muscle in self.muscle_list:  # run through the muscle list

            # load saved muscle ma data file
            with open(self.save_path + '/' + muscle + '.pkl', "rb") as muscle_file:
                muscle_para = pickle.load(muscle_file)
            
            joints_uniq = muscle_para['jit_uniq']     
            mjc_jnt_arr = muscle_para['mjc_jit_list_set']
            act_arr = muscle_para['act_list']
            
            # calcualte motion range based on fiber_opt and tendon_slack 
            fiber_opt = muscle_para['mtu_par_set']['fiber_opt']
            tendon_slack = muscle_para['mtu_par_set']['tendon_sla']
            
            fiber_len_range = np.zeros(2)
            
            # use the osim mtu lengths to calculate the mujoco muscle parameters
            osim_mtu_len_arr = np.array(muscle_para['mtu_length_osim'], copy=True)

            # extract true min/max across all sampled postures (handles extensor & multi-DOF muscles)
            min_osim_mtu_len = float(np.min(osim_mtu_len_arr))
            max_osim_mtu_len = float(np.max(osim_mtu_len_arr))
            if min_osim_mtu_len == max_osim_mtu_len:
                min_osim_mtu_len -= 0.005
                max_osim_mtu_len += 0.005
            
            # calculate relative fiber lengths
            fiber_len_range[0] = (min_osim_mtu_len - tendon_slack)/fiber_opt
            fiber_len_range[1] = (max_osim_mtu_len - tendon_slack)/fiber_opt
            
            # make sure that the fiber length range is ordered and within [0.01, 1.99]
            fiber_len_range[0] = np.clip(fiber_len_range[0], 0.01, 1.95)
            fiber_len_range[1] = np.clip(fiber_len_range[1], max(fiber_len_range[0] + 0.02, 0.05), 1.99)
            
            muscle_inst = self.mjc_model.actuator(muscle) #mujoco.mj_name2id(self.mjc_model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)
            
            fmax = muscle_para['mtu_par_set']['fmax']
            
            osimFP = muscle_para['mtu_force_osim']/fmax
            
            # actual muscle length
            muscle_inst.lengthrange[0] = min_osim_mtu_len
            muscle_inst.lengthrange[1] = max_osim_mtu_len
            
            # operation range (nomalized by L0)
            muscle_inst.gainprm[0:2] = fiber_len_range

            # initialize physiological Hill active force-length curve bounds [0.5, 1.6]
            muscle_inst.gainprm[4] = 0.5
            muscle_inst.gainprm[5] = 1.6
      
            # biasprm also NEEDS to be updated
            muscle_inst.biasprm[:] = muscle_inst.gainprm
                    
            err_ind, mjc_mtu_length, cost_org = getMuscleForceDiff(self.mjc_model, muscle,\
                                                              joints_uniq, mjc_jnt_arr,\
                                                              act_arr,\
                                                              osimFP)
            
            if err_ind:

                logger.info(f"Muscle : {muscle} ")
                
                # set the actual muscle lengths based on the mesh points

                if mjc_mtu_length.min() == mjc_mtu_length.max():
                    min_mjc_mtu_length = mjc_mtu_length.min() - 0.005
                    max_mjc_mtu_length = mjc_mtu_length.max() + 0.005
                else:
                    min_mjc_mtu_length = mjc_mtu_length.min()
                    max_mjc_mtu_length = mjc_mtu_length.max()

                muscle_inst.lengthrange[0] = min_mjc_mtu_length
                muscle_inst.lengthrange[1] = max_mjc_mtu_length
                
                # initialize lmin and lmax at physiological Hill defaults [0.5, 1.6]
                muscle_inst.gainprm[4:6] = [0.5, 1.6]
                muscle_inst.biasprm[4:6] = [0.5, 1.6]

                # save current mujoco model into cvt3...
                cvt3_model_path = self.mjc_model_path[0:-8] + 'cvt3.xml'
                mujoco.mj_saveLastXML(cvt3_model_path, self.mjc_model)

                # Jointly optimize [range_min, range_max, lmin, lmax, fpmax, fmax_scale]
                # so compliant-tendon muscles are not pinned at range[1] >= 1.0 or lmin=0, lmax=2 (#45)
                optParam_lb = [0.01, 0.05, 0.2, 1.05, 0.01, 0.5]
                optParam_ub = [1.90, 1.99, 0.95, 1.99, 5.0, 2.0]
                
                opt_results, self.mjc_model = fmOptPSO_cust(cvt3_model_path, muscle, joints_uniq,\
                                                       mjc_jnt_arr, act_arr, osimFP,\
                                                       optParam_lb, optParam_ub,\
                                                       cost_org)

                muscle_para['opt_results'] = opt_results

            else:  # if the error is smaller than threshold, then directly save the results

                logger.info('    Force errors between Osim and Mjc models are smaller than throshold, optimization skipped')

                muscle_para['opt_results'] = {'cost_opt': cost_org, 'cost_org': cost_org, 'res_opt': None} 


            # save the data based to the muscle parameter file
            with open(self.save_path + '/' + muscle + '.pkl', 'wb') as muscle_file_saving:
                pickle.dump(muscle_para, muscle_file_saving)

            # extract muscle instance again.
            muscle_inst = self.mjc_model.actuator(muscle)
            
            # change fmax back to unnormalized force
            res_opt = muscle_para['opt_results']['res_opt']
            if res_opt is not None and len(res_opt) > 0:
                muscle_inst.gainprm[2] = res_opt[-1]*fmax
            else:
                muscle_inst.gainprm[2] = fmax
                
            # change vmax to 10*L0, and fvmax as 1.4 (opensim default)
            muscle_inst.gainprm[6] = 10
            muscle_inst.gainprm[8] = 1.4
            
            # copy the parameter from gainprm to biasprm
            muscle_inst.biasprm[:] = muscle_inst.gainprm

        # save the updated model
        cvt3_model_path = self.mjc_model_path[0:-8] + 'cvt3.xml'
        mujoco.mj_saveLastXML(cvt3_model_path, self.mjc_model)
            
        return cvt3_model_path 
            
        
    def compMuscleForceResults(self, mjc_model_path = None):

        # plot the muscle force errors before and after optimization (bar plot)
        # from the saved muscle force optimization results

        os.makedirs(self.save_path + '/muscle_forces', exist_ok = True)

        # load mujoco model if given.
        if mjc_model_path:
            mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)
        else:
            mjc_model_path = self.mjc_model_path[0:-8] + 'cvt3.xml'
            mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)

        # if muscule list provided, only compare the results of them
        if self.muscle_list:
            muscle_list = self.muscle_list
        else:
            # Scan the result folder to find all the <muscle_name>.pkl files and plot all of them
            excluded_pkls = {"config", "overall_comp_muscleforces"}
            muscle_list = [
                os.path.split(f)[1][0:-4]
                for f in glob.glob(self.save_path + "/*.pkl")
                if os.path.split(f)[1][0:-4] not in excluded_pkls
            ]

        rms_org = []
        rms_opt = []

        for i_mus, muscle in enumerate(muscle_list):
            # load the mus_para data from the saved files
            with open(self.save_path + '/' + muscle + '.pkl', "rb") as muscle_file:
                muscle_para_opt = pickle.load(muscle_file)

            # generate force-length comparison plot for each muscle
            mtu_force_osim = muscle_para_opt['mtu_force_osim']
            mtu_force_osim_passive = muscle_para_opt['pas_force_osim']
            mtu_length_osim = muscle_para_opt['mtu_length_osim']
            joints = muscle_para_opt['jit_uniq']
            # osim_jnt_arr = muscle_para_opt['jit_list_set']
            mjc_jnt_arr = muscle_para_opt['mjc_jit_list_set']
            act_arr = muscle_para_opt['act_list']

            # generate muscle force length curves in mujoco
            mtu_force_mjc, mtu_length_mjc =\
            getMuscleForceLengthCurvesSim(mjc_model, muscle,\
                                        joints, mjc_jnt_arr, act_arr)
                                        
            # get the passive muscle force length curves in mujoco
            mtu_force_mjc_passive, mtu_length_mjc_passive =\
            getMuscleForceLengthCurvesSim(mjc_model, muscle,\
                                        joints, mjc_jnt_arr, [0])                             
                                    
            # plot the force comparison curves
            self.curveplotForceLength(muscle, mtu_length_osim, mtu_force_osim,\
                                    mtu_force_osim_passive, mtu_length_mjc, mtu_length_mjc_passive,\
                                    mtu_force_mjc, mtu_force_mjc_passive, act_arr)
            
            # save mjc forve lenth curve as well
            muscle_para_opt['mtu_force_mjc'] = mtu_force_mjc
            muscle_para_opt['mtu_force_mjc_passive'] = mtu_force_mjc_passive
            muscle_para_opt['mtu_length_mjc'] = mtu_length_mjc
            muscle_para_opt['mtu_length_mjc_passive'] = mtu_length_mjc_passive

            with open(self.save_path + '/' + muscle + '.pkl', "wb") as muscle_file:
                pickle.dump(muscle_para_opt, muscle_file)

            # plot the overall comparison results when 'opt_results' exists.
            if 'opt_results' in muscle_para_opt.keys():
                # save overall errors 
                rms_org.append(muscle_para_opt['opt_results']['cost_org'])
                rms_opt.append(muscle_para_opt['opt_results']['cost_opt'])

                # save the overall rms differences between forces
                rms_saving = {}
                rms_saving['rms_org'] = rms_org
                rms_saving['rms_opt'] = rms_opt
                rms_saving['rms_opt_mean'] = np.mean(rms_opt)
                rms_saving['rms_opt_std'] = np.std(rms_opt)

                # save the data based to the muscle parameter file
                with open(self.save_path + '/overall_comp_muscleforces.pkl', 'wb') as rms_saving_file:
                    pickle.dump(rms_saving, rms_saving_file)

        # plot the overall comparison results when 'opt_results' exists.
        if len(rms_org) > 0:
            # bar plots of the muscle force differences
            self.barplotMF(muscle_list, rms_org, rms_opt)

    def curveplotForceLength(self, muscle, length_mtu_osim, mtu_force_osim, mtu_force_osim_passive,\
                             length_mtu_mjc, mtu_length_mjc_passive, mjc_force_mjc,\
                             mtu_force_mjc_passive, act_arr):
        """
        Plot the muscle force length curves        
        """

        actMesh = len(act_arr)

        f = plt.figure(figsize=(10, 8))
        ax1 = f.add_subplot(1, 2, 1)
        ax2 = f.add_subplot(1, 2, 2)

        # find the minimal and maximum values in mjc and osim mf data, to set the axis equal
        # osim and mjc model have oppsite signs in muscle forces
        max_mf = np.maximum(mtu_force_osim.max(), -mjc_force_mjc.min())
        min_mf = np.minimum(np.minimum(mtu_force_osim.min(), -mjc_force_mjc.max()), 0)
        
        max_ml = np.maximum(length_mtu_osim.max(), length_mtu_mjc.max())
        min_ml = np.minimum(length_mtu_osim.min(), length_mtu_mjc.min())

        osim_accend_index = np.argsort(length_mtu_osim)
        
        for c in range(actMesh):
            line_color = tuple([c, c, actMesh]/np.sqrt(2*c**2 + actMesh**2))

            ax1.plot(length_mtu_osim[osim_accend_index]*100, mtu_force_osim[c][osim_accend_index], marker = "s", color=line_color)
            ax1.plot(length_mtu_osim[osim_accend_index]*100, mtu_force_osim_passive[c][osim_accend_index], linestyle = "dashed",\
                        marker = "o", color=line_color)

        ax1.set_ylabel("Muscle forces (N)")
        ax1.set_xlabel(" MTU length (cm)")
        ax1.set_ylim([0.95*min_mf, 1.05*max_mf])
        ax1.set_xlim([0.95*min_ml*100, 1.05*max_ml*100])
        ax1.set_title("OSIM")

        for c in range(actMesh):
            line_color = tuple([c, c, actMesh]/np.sqrt(2*c**2 + actMesh**2))
            mjc_accend_index = np.argsort(length_mtu_mjc[c])
            pas_idx = c if c < len(mtu_length_mjc_passive) else 0
            mjc_accend_passive_index = np.argsort(mtu_length_mjc_passive[pas_idx])
            ax2.plot(length_mtu_mjc[c][mjc_accend_index]*100, -mjc_force_mjc[c][mjc_accend_index], marker = "s", color=line_color)
            ax2.plot(mtu_length_mjc_passive[pas_idx][mjc_accend_passive_index]*100, -mtu_force_mjc_passive[pas_idx][mjc_accend_passive_index], linestyle = "dashed",\
                        marker = "o", color=line_color)

        plt.legend(["Total force", "Passive force"])
        ax2.set_ylabel("Muscle forces (N)")
        ax2.set_xlabel(" MTU length (cm)")
        ax2.set_ylim([0.95*min_mf, 1.05*max_mf])
        ax2.set_xlim([0.95*min_ml*100, 1.05*max_ml*100])
        ax2.set_title("MJC")

        plt.suptitle(muscle)

        f.savefig(self.save_path + '/muscle_forces/' + muscle + '.svg', format = "svg")
        plt.close(f)

    def barplotMF(self, muscle_list, rms_org, rms_opt):
        """"
        Bar plot to compare the overall muscle force errors
        """

        fig = plt.figure(figsize=(10, 8))

        ax = fig.add_subplot(111)
        X = np.arange(len(muscle_list))
        ax.bar(X - 0.15, np.array(rms_org), 0.25, label='Default')
        ax.bar(X + 0.15, np.array(rms_opt), 0.25, label='Optimized')
        plt.ylabel('RMS 1/Fmax')
        plt.xticks(X, muscle_list)
        plt.xticks(rotation=90)
        ax.legend()
        # plt.show()
        fig.savefig(self.save_path  + '/muscle_forces/overall_comp_barplot.svg', format = "svg")
        plt.close(fig)
        
        
        
            
        
            
            
