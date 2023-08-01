#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  5 22:29:33 2022

@author: hwang
"""

from myoconverter.optimization.utils.UtilsLengthOpt import getMomentArmDiff, maOptPSO_cust
from myoconverter.optimization.model_states.OsimMuscleStates import OsimMuscleStates
from myoconverter.optimization.utils.UtilsMujoco import sortMuscleWrapSiteJoint, \
      computeMomentArmMuscleJoints, getCoordinateRange_mjc
from loguru import logger
from numpy import pi
import pickle
import os
import numpy as np
import mujoco
import opensim
import seaborn as sns
import matplotlib.pyplot as plt 
import glob


class MomentArmOpt:
    """
    Class to optimize muscle moment arms of mujoco model
    """
    
    def __init__(self, mjc_model_path, osim_model_path, save_path, muscle_list = None,\
                 osim_data_overwrite = False, speedy = False):
                 
        """
        Parameters
        ----------
        mjc_model_path : string
            The model path of mjc model
            
        osim_model_path : string
            The model path of osim model
            
        save_path : string
            The path to save moment arm results
            
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
        
        # load mujoco model
        self.mjc_model_path = mjc_model_path
        self.mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)
        # load osim model
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
                
        # saving path has to be valid
        try:
            os.makedirs(save_path, exist_ok = True)
         
        except OSError:
            raise("Please provide a valid saving path")
            
        self.save_path = save_path
        self.speedy = speedy
            
        # Check if the osim_data needs to be over write or not.
        # If so, generate them using the modelState/OsimMuscleStates class
        # Another case to regenerate is that there is no data file saved 
        # in the saving path of the provided muscle

        # get mjc joint ranges
        ang_ranges_mjc, free_jnt_id_mjc = getCoordinateRange_mjc(self.mjc_model)

        if osim_data_overwrite:

            logger.info("Overwrite command confirmed")
            logger.info("Generating MA data from OsimMuscleStates, may take a while")

            # This will take a while, previous generated MA data files of selected muscles will be overwrited
            osim_mus_sta = OsimMuscleStates(self.osim_model, self.muscle_list)

            # regenerate moment arms from osim model
            osim_mus_sta.getMomentArms(save_path = self.save_path,\
                                       speedy = speedy)
            
            for mus_name in self.muscle_list:
                # add mujoco joint range information, in case the joint range are different.
                # load saved muscle ma data file
                with open(self.save_path + '/' + mus_name + '.pkl', "rb") as muscle_file:
                    muscle_para_ma = pickle.load(muscle_file)
                
                # run through the wrapping coordinates
                mjc_motion_ranges = []
                for joints in muscle_para_ma['wrapping_coordinates']:
                    mjc_motion_range = []
                    for joint in joints:
                        mjc_motion_range.append(ang_ranges_mjc[joint])
                    mjc_motion_ranges.append(mjc_motion_range)

                muscle_para_ma['mjc_coordinate_ranges'] = mjc_motion_ranges

                # save moment arm data if a saving path is provided
                with open(self.save_path + '/' + mus_name + '.pkl', 'wb') as muscle_saving:
                    pickle.dump(muscle_para_ma, muscle_saving)
                    muscle_saving.close()
            
        else:

            logger.info("Overwrite not required")
            logger.info("checking if the muscle data file exist")

            # Overwrite not required, checking if the muscle data file exist
            # If not, will be regenerated
            for mus_name in self.muscle_list:

                if not os.path.isfile(self.save_path + '/' + mus_name + '.pkl'):
                    logger.info(f"Muscle: {mus_name} data file does not exist, regenerating")

                    osim_mus_sta = OsimMuscleStates(self.osim_model, mus_name)
                    # regenerate moment arms
                    osim_mus_sta.getMomentArms(save_path = self.save_path,\
                                            speedy = speedy)
                    
                    # add mujoco joint range information, in case the joint range are different.
                                # load saved muscle ma data file
                    with open(self.save_path + '/' + mus_name + '.pkl', "rb") as muscle_file:
                        muscle_para_ma = pickle.load(muscle_file)

                    # run through the wrapping coordinates
                    mjc_motion_ranges = []
                    for joints in muscle_para_ma['wrapping_coordinates']:
                        mjc_motion_range = []
                        for joint in joints:
                            mjc_motion_range.append(ang_ranges_mjc[joint])
                        mjc_motion_ranges.append(mjc_motion_range)

                    muscle_para_ma['mjc_coordinate_ranges'] = mjc_motion_ranges

                    # save moment arm data if a saving path is provided
                    with open(self.save_path + '/' + mus_name + '.pkl', 'wb') as muscle_saving:
                        pickle.dump(muscle_para_ma, muscle_saving)
                        muscle_saving.close()

                else:
                    logger.info(f"Muscle: {mus_name} data file exist, will reuse it in the optimization")

        logger.info('Finished cvt2 initialize')


    def optMomentArms(self):

        """
        This function optimize wrapping sites in order to have matching moment
        arms with a given data set (osim_ma_data)
        
        Parameters
        ----------
        
        Returns
        -------
        cvt2_model_path: string
            The optimized #####_cvt2.xml model path
        """
        
        logger.info('Running MA optimization with the given muscle list one by one')            
            
        for muscle in self.muscle_list:

            logger.info(f"Muscle : {muscle} ")

            # load saved muscle ma data file
            with open(self.save_path + '/' + muscle + '.pkl', "rb") as muscle_file:
                muscle_para_osim = pickle.load(muscle_file)

            # add information of wrapping objects and site points 
            muscle_para = sortMuscleWrapSiteJoint(self.mjc_model, muscle_para_osim)

            opt_res_list = len(muscle_para['wrapping_coordinates'])*[None]  # initilize results list

            for i_jnt, joints in enumerate(muscle_para['wrapping_coordinates']):  # run through the joint combinations

                logger.info(f"    At joints: {joints[0]}")

                # check if MA differences are larger than the thresholds
                # absolulte error larger than 0.001 and relative error larger than 5%
                err_ind, cost_org = getMomentArmDiff(self.mjc_model,\
                                                        muscle,
                                                        joints,\
                                                        muscle_para['mjc_coordinate_ranges'][i_jnt],\
                                                        muscle_para['osim_ma_data'][i_jnt],\
                                                        muscle_para['evalN'][i_jnt])

                if not err_ind:
                    logger.info('    MA errors between Osim and Mjc models are smaller than throshold, optimization skipped')
                    # still save the optimization data 
                    opt_res_list[i_jnt] = {'cost_org': cost_org, 'cost_opt': cost_org, 'par_opt': []}

                    continue
                    
                else:
                    logger.info('    Start MA optimization')


                if not muscle_para['wrapping_info'][i_jnt]:  # if the wrapping information is None, do not run optimization
                    logger.info('    No wrapping information extracted, skipped')
                    # still save the optimization data
                    opt_res_list[i_jnt] = {'cost_org': cost_org, 'cost_opt': cost_org, 'par_opt': []}

                    continue

                else:
                    wrap_info = muscle_para['wrapping_info'][i_jnt]  # extract the wrapping information 
                    # site_info = muscle_para['wrapping_info'][i_jnt][-1]  # Right now, assume just one affecting wrapping point

                    # extract wrapping info
                    wrap_name = wrap_info[0]
                    wrap_id = wrap_info[1]
                    pos_wrap = wrap_info[2]
                    size_wrap = wrap_info[3]
                    rot_wrap = wrap_info[4]
                    wrap_type = wrap_info[5]
                    side_id = wrap_info[6][0]
                    side_pos = wrap_info[6][1]
                            
                    # initilize the optimizing parameters
                    optParam_lb, optParam_ub = [], []

                    if 'torus' in wrap_info[0]:
                        logger.info('    Wrapping object type is Torus, optimization skipped')

                        # if torus, then do nothing, since muscle is already going through
                        # the wrapping object, which is sphere. Just save empty results

                        # save opt results
                        opt_res_list[i_jnt] = [{'cost_org': cost_org, 'cost_opt': cost_org, 'par_opt': side_pos}]

                        continue

                    elif 'cylinder' in wrap_info[0]:
                    # do normal optimzation of the sidesite location

                        # save the updated model
                        cvt2_model_path = self.mjc_model_path[0:-8] + 'cvt2.xml'
                        with open(cvt2_model_path, 'w+') as xml_file:
                            mujoco.mj_saveLastXML(cvt2_model_path, self.mjc_model)

                        wrap_type_opt = 'CYLINDER'
                                
                        # rotation bounds, only optimize the rotation location of the site
                        optParam_lb.append(-pi)  
                        optParam_ub.append(pi)
                        
                        optParam_lb.append(-size_wrap[1])  
                        optParam_ub.append(size_wrap[1])
                        
                        # run optimization with PSO optimizer
                        opt_site, self.mjc_model  = maOptPSO_cust(cvt2_model_path, muscle, joints,\
                                                            muscle_para['mjc_coordinate_ranges'][i_jnt],\
                                                            side_id, wrap_type_opt, wrap_id, pos_wrap,\
                                                            size_wrap, rot_wrap,\
                                                            muscle_para['osim_ma_data'][i_jnt],\
                                                            muscle_para['evalN'][i_jnt],\
                                                            optParam_lb, optParam_ub, cost_org, speedy = self.speedy)
                    
                    
                        opt_res_list[i_jnt] = opt_site  # save opt results

                    elif 'sphere' in wrap_info[0]:
                    # do normal optimzation of the sidesite location

                        # save the updated model                      
                        cvt2_model_path = self.mjc_model_path[0:-8] + 'cvt2.xml'
                        with open(cvt2_model_path, 'w+') as xml_file:
                            mujoco.mj_saveLastXML(cvt2_model_path, self.mjc_model)

                        wrap_type_opt = 'SPHERE'
                                     
                        optParam_lb.append(-pi)  # rotation bounds
                        optParam_ub.append(pi)
                                
                        # second rotation bounds
                        optParam_lb.append(0)
                        optParam_ub.append(pi)
                        
                        # run optimization with PSO optimizer
                        opt_site, self.mjc_model  = maOptPSO_cust(cvt2_model_path, muscle, joints,\
                                                            muscle_para['mjc_coordinate_ranges'][i_jnt],\
                                                            side_id, wrap_type_opt, wrap_id, pos_wrap,\
                                                            size_wrap, rot_wrap,\
                                                            muscle_para['osim_ma_data'][i_jnt],\
                                                            muscle_para['evalN'][i_jnt],\
                                                            optParam_lb, optParam_ub, cost_org, speedy = self.speedy)
                        
                        
                        opt_res_list[i_jnt] = opt_site  # save opt results

                    elif 'ellipsoid' in wrap_info[0]:
                    # do optimization of both sidesite and ellipsoid sizes

                        # if the wrapping objects were converted from ellipsoid, then
                        # its position and rotation may need optimized to have the 
                        # best MA matches
                    
                        if wrap_type == 'CYLINDER':  # check converted wrap type

                            # save the updated model                            
                            cvt2_model_path = self.mjc_model_path[0:-8] + 'cvt2.xml'
                            with open(cvt2_model_path, 'w+') as xml_file:
                                mujoco.mj_saveLastXML(cvt2_model_path, self.mjc_model)
                            
                            wrap_type_opt = 'ELLIPSOID_CYLINDER'

                            opt_range = 0.25  # range of the wrap object size change
                        
                            # bounds of cylinder radius
                            optParam_lb.append((1 - opt_range)*size_wrap[0])
                            optParam_ub.append((1 + opt_range)*size_wrap[0])
                        
                            # bounds of cylinder lengths
                            optParam_lb.append((1 - opt_range)*size_wrap[1])
                            optParam_ub.append((1 + opt_range)*size_wrap[1])
                        
                            # bounds of cylinder position, in the range of radius change
                            for pos in pos_wrap:
                                optParam_lb.append(pos - opt_range*size_wrap[0])
                                optParam_ub.append(pos + opt_range*size_wrap[0])
                            
                            optParam_lb.append(-pi)  # rotation bounds of sidesite
                            optParam_ub.append(pi)
                            
                            optParam_lb.append(-size_wrap[1])  # the translation bounds
                            optParam_ub.append(size_wrap[1])
                                                    
                            # run optimization with PSO optimizer
                            opt_site, self.mjc_model  = maOptPSO_cust(cvt2_model_path, muscle, joints,\
                                                            muscle_para['mjc_coordinate_ranges'][i_jnt],\
                                                            side_id, wrap_type_opt, wrap_id, pos_wrap,\
                                                            size_wrap, rot_wrap,\
                                                            muscle_para['osim_ma_data'][i_jnt],\
                                                            muscle_para['evalN'][i_jnt],\
                                                            optParam_lb, optParam_ub, cost_org, speedy = self.speedy)

                            opt_res_list[i_jnt] = opt_site  # save opt results

                        if wrap_type == 'SPHERE':

                            # save the updated model
                            cvt2_model_path = self.mjc_model_path[0:-8] + 'cvt2.xml'
                            with open(cvt2_model_path, 'w+') as xml_file:
                                mujoco.mj_saveLastXML(cvt2_model_path, self.mjc_model)
                        
                            wrap_type_opt = 'ELLIPSOID_SPHERE'
                        
                            opt_range = 0.25  # range of the wrap object size change
                        
                            # bounds of sphere radius
                            optParam_lb.append((1 - opt_range)*size_wrap[0])
                            optParam_ub.append((1 + opt_range)*size_wrap[0])
                            
                            # bounds of sphere position, in the range of radius change
                            for pos in pos_wrap:
                                optParam_lb.append(pos - opt_range*size_wrap[0])
                                optParam_ub.append(pos + opt_range*size_wrap[0])
                            
                            optParam_lb.append(-pi)  # rotation bounds
                            optParam_ub.append(pi)
                            
                            optParam_lb.append(0)  # second rotation bounds
                            optParam_ub.append(pi)
                            
                            # run optimization with PSO optimizer
                            opt_site, self.mjc_model  = maOptPSO_cust(cvt2_model_path, muscle, joints,\
                                                            muscle_para['mjc_coordinate_ranges'][i_jnt],\
                                                            side_id, wrap_type_opt, wrap_id, pos_wrap,\
                                                            size_wrap, rot_wrap,\
                                                            muscle_para['osim_ma_data'][i_jnt],\
                                                            muscle_para['evalN'][i_jnt],\
                                                            optParam_lb, optParam_ub, cost_org, speedy = self.speedy)

                            opt_res_list[i_jnt] = opt_site  # save opt results

                    else:
                        logger.debug("Unknow wrapping type detected, error occur")
                        raise('Unknow wrapping type detected, error occur, please check... \n')

            muscle_para['opt_results'] = opt_res_list

            # save the data based to the muscle parameter file
            muscle_file_saving = open(self.save_path + '/' + muscle + '.pkl', 'wb')
            pickle.dump(muscle_para, muscle_file_saving)
            muscle_file_saving.close()

        # save the updated model
        cvt2_model_path = self.mjc_model_path[0:-8] + 'cvt2.xml'
        with open(cvt2_model_path, 'w+') as xml_file:
            mujoco.mj_saveLastXML(cvt2_model_path, self.mjc_model)

        return cvt2_model_path
    
    def compMomentArmResults(self, mjc_model_path = None):
        """
        Plot the moment arm results before and after the above optimization step

        Parameters
        ----------
        mjc_model_path: string, optional
            mujoco model path
        
        Returns
        -------
        None.

        """

        os.makedirs(self.save_path + '/moment_arms', exist_ok = True)

        # load mujoco model if given.
        if mjc_model_path:
            mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)
        else:
            mjc_model_path = self.mjc_model_path[0:-8] + 'cvt2.xml'
            mjc_model = mujoco.MjModel.from_xml_path(mjc_model_path)

        if self.muscle_list:
            muscle_list = self.muscle_list  # if selected muscle given, only validate these
        else:
            # Scan the result folder to find all the <muscle_name>.pkl files and plot all of them
            muscle_list = [os.path.split(f)[1][0:-4] for f in glob.glob(self.save_path + "/*.pkl")]

        # find all the joints from mjc model
        joint_list = [mujoco.mj_id2name(mjc_model, mujoco.mjtObj.mjOBJ_JOINT, idx) for idx in range(mjc_model.njnt)]

        rMax = len(muscle_list)
        cMax = len(joint_list)
        
        cost_org_mat = np.zeros((rMax, cMax))
        cost_opt_mat = np.zeros((rMax, cMax))
        
        for i_mus, muscle in enumerate(muscle_list):
            # load the mus_para data from the saved files
            muscle_file = open(self.save_path + '/' + muscle + '.pkl', "rb+")
            muscle_para_opt = pickle.load(muscle_file)

            # Generate the moment arm curves for comparison plots. Moment arm of Osim model
            # will just use the reference data that pre-generated for optimization. Moment arm
            # of Mjc model will be regenerated to increase the mesh density. This is to check
            # if there are any random jumpping paths (that were not covered by the optimization)

            mjc_ma_data= []

            for ij, joints in enumerate(muscle_para_opt['wrapping_coordinates']):

                nEval = muscle_para_opt['evalN'][ij]
                nJnt = len(joints)
                jntRanges_mjc = muscle_para_opt['mjc_coordinate_ranges'][ij]
                jntRanges_osim = muscle_para_opt['osim_coordinate_ranges'][ij]

                # sort the osim moment arms into matrices
                ma_mat_osim = self.maVectorSort(muscle_para_opt['osim_ma_data'][ij], nJnt, nEval)

                # calculate mujoco moment arms and sort it into matrices
                ma_vec_mjc = computeMomentArmMuscleJoints(mjc_model, muscle, joints,\
                                                    jntRanges_mjc, nEval)
                
                ma_mat_mjc = self.maVectorSort(ma_vec_mjc, nJnt, nEval)

                # plot the moment arm plots
                self.individualMuscleMAPlot(muscle, joints, nJnt, jntRanges_osim, jntRanges_mjc, \
                                            ma_mat_osim, ma_mat_mjc, nEval, nEval)
                
                mjc_ma_data.append(ma_vec_mjc)
                        
                for joint in joints:
                    if not joint in joint_list:
                        joint_list.append(joint)

                    jnt_index = joint_list.index(joint)

                    if muscle_para_opt['opt_results'][ij]:  # if results exist
                        cost_org_mat[i_mus, jnt_index] = muscle_para_opt['opt_results'][ij]['cost_org']
                        cost_opt_mat[i_mus, jnt_index] = muscle_para_opt['opt_results'][ij]['cost_opt']

            muscle_para_opt['mjc_ma_data'] = mjc_ma_data

            pickle.dump(muscle_para_opt, muscle_file)
            muscle_file.close()

        # save the overall rms differences between moment arms
        rms_saving = {}
        rms_saving['cost_org_mat'] = cost_org_mat
        rms_saving['cost_opt_mat'] = cost_opt_mat
        nonzero_error_array = cost_opt_mat[cost_opt_mat != 0]
        rms_saving['cost_opt_mat_mean'] = np.mean(nonzero_error_array)
        rms_saving['cost_opt_mat_std'] = np.std(nonzero_error_array)

        # save the data based to the muscle parameter file
        rms_saving_file = open(self.save_path + '/overall_comp_momentarms.pkl', 'wb')
        pickle.dump(rms_saving, rms_saving_file)
        rms_saving_file.close()

        # generate the overall heat map                 
        self.heatMap(muscle_list, joint_list, cost_org_mat, cost_opt_mat)

    def maVectorSort(self, ma_vec, nJnt, nEval):
        """
        This function sort the moment arm vector that generated by optimization procedures.
        Results of this sort function are joint specific lists that contains the moment arms
        of it. The number of list depends on the mesh points of the other coupled joints. The
        length of each list represent the checking points of this joint angle.

        In the optimization, itertools.product will generate the angle mesh list in this structure:
          [Ang11, Ang21, Ang31], [Ang11, Ang21, Ang32], ...
          [Ang11, Ang22, Ang31], [Ang11, Ang22, Ang32], ...
          ...
          [Ang12, Ang21, Ang31], [Ang12, Ang21, Ang32], ...
          ...
          ...
        """
        
        # run through each joint
        ma_mat = np.zeros((nJnt, nEval, nEval**(nJnt-1)))

        for nj in range(nJnt):  # run through joints
            njVec = ma_vec[nj]

            for ne in range(nEval):  # run through evaulation joints
                for ne2 in range(nEval**(nJnt-1)):
                    ma_mat[nj, ne, ne2] = ma_vec[ne*nEval**(nJnt-1) + ne2][nj]

        return ma_mat


    def individualMuscleMAPlot(self, muscle, joints, nJnt, joint_ranges_osim, joint_ranges_mjc, ma_mat_osim, ma_mat_mjc, nEval_osim, nEval_mjc):
        """"
        Plot individual muscle moment arm curves. Osim and Mjc are plotted side by side
        for the situation that multiple joints are coupling in the plot joint moment arm.
        In this case, a number of mesh points are checked on these coupling joints. And in
        this case, plot Osim and Mjc moment arms on top of each other.
        """

        # super title
        supTitle = muscle
        for joint in joints:
            supTitle = supTitle + " - " + joint
        
        for ij, joint in enumerate(joints):

            joint_range_osim = joint_ranges_osim[ij]
            joint_range_mjc = joint_ranges_mjc[ij]
            
            f = plt.figure(figsize=(10, 8))

            ax1 = f.add_subplot(1, 2, 1)
            ax2 = f.add_subplot(1, 2, 2)

            x_osim = np.linspace(joint_range_osim[0], joint_range_osim[1], nEval_osim, endpoint=True)
            x_mjc = np.linspace(joint_range_mjc[0], joint_range_mjc[1], nEval_mjc, endpoint=True)

            # find the minimal and maximum values in mjc and osim ma data, to set the axis equal
            # osim and mjc model have oppsite signs in moment arms
            max_ma = np.maximum(ma_mat_osim[ij, :, :].max(), -ma_mat_mjc[ij, :, :].min())
            min_ma = np.minimum(ma_mat_osim[ij, :, :].min(), -ma_mat_mjc[ij, :, :].max())
           
            for c in range(nEval_osim**(nJnt-1)):
                line_color = tuple([c, c, nEval_mjc**(nJnt-1)]/np.sqrt(2*c**2 + (nEval_mjc**(nJnt-1))**2))
                ax1.plot(x_osim, ma_mat_osim[ij, :, c]*100, marker = "s", color=line_color)

            ax1.set_ylabel("moment arms (cm)")
            ax1.set_xlabel(joint + " (rad)")
            ax1.set_ylim([min_ma*100, max_ma*100])
            ax1.set_title("OSIM")

            for c in range(nEval_mjc**(nJnt-1)):
                line_color = tuple([c, c, nEval_mjc**(nJnt-1)]/np.sqrt(2*c**2 + (nEval_mjc**(nJnt-1))**2))
                ax2.plot(x_mjc, -ma_mat_mjc[ij, :, c]*100, marker = "s", color=line_color)

            ax2.set_ylabel("moment arms (cm)")
            ax2.set_xlabel(joint + " (rad)")
            ax2.set_ylim([min_ma*100, max_ma*100])
            ax2.set_title("MJC")

            plt.suptitle(supTitle)

            f.savefig(self.save_path + '/moment_arms/' + muscle + '_' + joint + '.svg', format = "svg")
            plt.close(f)


    def heatMap(self, x_list, y_list, mat_1, mat_2):
        """
        Generate the moment arm heat map comparison.
        """
        
        maxV = max(mat_1.max(), mat_2.max())*100  # change to cm
        
        # plot the heat mapping
        sns.set_theme(style = "white")
        f = plt.figure(figsize=(10, 8))
        ax1 = f.add_subplot(121)
        cmap = sns.diverging_palette(230, 20, as_cmap=True)
        sns.heatmap(mat_1[0:len(x_list), 0:len(y_list)]*100, cmap = cmap, vmax= maxV,\
                    center = 0, square = True, linewidths=0.5, cbar_kws={"shrink": 0.5})
            
        xtick = np.linspace(0, len(x_list), len(x_list), endpoint=False, dtype=int) + 0.5
        ytick = np.linspace(0, len(y_list), len(y_list), endpoint=False, dtype=int) + 0.5

        ax1.set_xticks(ytick)
        ax1.set_yticks(xtick)
        ax1.set_xticklabels(y_list)
        ax1.set_yticklabels(x_list)
        plt.xticks(rotation=90)
        plt.yticks(rotation=0)
        plt.title('Before Opt')
    
        ax2 = f.add_subplot(122)
        cmap = sns.diverging_palette(230, 20, as_cmap=True)
        sns.heatmap(mat_2[0:len(x_list), 0:len(y_list)]*100, cmap = cmap, vmax= maxV,\
                    center = 0, square = True, linewidths=0.5, cbar_kws={"shrink": 0.5})
            
        ax2.set_xticks(ytick)
        ax2.set_yticks(xtick)
        ax2.set_xticklabels(y_list)
        ax2.set_yticklabels(x_list)
        plt.xticks(rotation=90)
        plt.yticks(rotation=0)
        plt.title('After Opt')

        plt.suptitle("Moment arm comparison of all muscles (cm)")
        
        f.savefig(self.save_path + '/moment_arms/overall_comp_heatmap.svg', format = "svg")
        plt.close(f)