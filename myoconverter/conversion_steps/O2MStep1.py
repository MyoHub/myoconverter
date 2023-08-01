#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This script includes all the steps of transfering an OpenSim model to Mujoco.

Created on Tue Jul 27 23:19:13 2021

@author: hwang
"""

# import the the first step model converter
from myoconverter.xml.converter import convert  

# import the end point calculation functions for the kinematics check
from myoconverter.optimization.utils.UtilsMujoco import calculateEndPoints_mjc  
from myoconverter.optimization.utils.UtilsOpensim import calculateEndPoints_osim, extractMarkerSet
from loguru import logger

import os
import numpy as np
import matplotlib.pyplot as plt
import pickle


class BasicModelConvert:
    """ Class convert OpenSim MSK model to MuJoCo xml format.
    
        Bodies, joints, muscles, ligaments, sites, constraints
        are all converted. 
        Forward kinematic check is perfomed for validating the
        accuracy of the bone shapes and joint definitions.
    
    """
    
    def __init__(self, osim_model_file, geometry_path, saving_path,\
                 add_ground_geom = False, treat_as_normal_path_point = False):
    
        """ Paremters
        
        osim_model_file: string
            The path of osim model to be converted.
            
        geometry_path: string
            The path of the osim model mesh files.
            
        saving_path: string
            The path to save converted model.

        add_ground_geom: boolean
            If true, add ground into the converted model

        treat_as_normal_path_point: boolean
            If true, treat moving and conditional points as fixed points
        
        """

        # check model file and path
        try: 
            os.path.isfile(osim_model_file)
        
        except OSError:
            raise("Cannot find the osim file, please check the model path & name")

        try: 
            os.path.isdir(geometry_path)
            
        except OSError:
            raise("Cannot find the geometry path folder")
        
        try:
            os.makedirs(saving_path, exist_ok = True) 
         
        except OSError:
            raise("Please provide a valid saving path")
        
        self.osim_model_file = osim_model_file
        self.geometry_path = geometry_path
        self.saving_path = saving_path
        self.add_ground_geom = add_ground_geom
        self.treat_as_normal_path_point = treat_as_normal_path_point
        
    def cvt1_ModelConvert(self):
        """ 
            Convert the osim model into mujoco using the functions in xml folder
        """
        
        # define the model saving path
        model_saving_path = self.saving_path[0:self.saving_path.rfind('/')]
        osim_file_name = self.osim_model_file[self.osim_model_file.rfind('/'):]

        kwargs = {}  # define kwargs inputs
        kwargs['geometry_folder'] = self.geometry_path
        kwargs['add_ground_geom'] = self.add_ground_geom
        kwargs['treat_as_normal_path_point'] = self.treat_as_normal_path_point

        ## convert the OpenSim model into MuJoCo
        self.converted_mjc_model_file = convert(self.osim_model_file, model_saving_path, **kwargs)
        
        return self.converted_mjc_model_file

    def vlt1_forwardKinematicsValidation(self, mjc_model_path = None, end_points_osim = None, speedy = False):
        """ Forward kinematic check
            Use markers at the surface of the body to compare the end point positions
            based on the same joint angle setups in both models.
            
            Parameters:
            
            mjc_model_path: string
                The mujoco model path.
                
            end_points_osim: list of strings
                A list of marker names that defined in the Osim model for the forward kinematic check
                
            speedy: boolean
                If True, run lower number of posture check [5 <--> 10].

        """

        os.makedirs(self.saving_path + '/end_points', exist_ok = True)
        
        # For validation, if the endpoints not provided, then extract all markers in the osim model
        if end_points_osim is None:
            logger.info("Extract marker names (as endpoints) from osim model")
            end_points_osim = extractMarkerSet(self.osim_model_file)

        # if mjc model not provided, then use the converted model
        if not mjc_model_path:
            if self.converted_mjc_model_file:
                mjc_model_path = self.converted_mjc_model_file
            else:
                raise('Cvt 1 model path not provided, please either run Cvt1, or provide the Cvt1 model path.')

        # add '_marker' to the osim marker names for mujoco model
        end_points_mjc = []
        for marker in end_points_osim:
            end_points_mjc.append(marker + '_marker')
            
        # choose evalN based on speedy setting
        if speedy:
            evalN = 5
        else:
            evalN = 10

        # get the marker location of two models when explore different postures
        logger.info("Get the endpoint locations by meshing the joint angles within the range limits")
        jnt_ranges_osim, end_pos_osim = calculateEndPoints_osim(self.osim_model_file, end_points_osim, evalN)
        jnt_ranges_mjc, end_pos_mjc = calculateEndPoints_mjc(mjc_model_path, end_points_mjc, evalN)
        
        # check the joionts, joint range, and the marker locaitons
        # first check if the list of joints are match nor not

        logger.info("Check if the joint motion ranges are the same between Osim and Mjc models")
        if not set(jnt_ranges_osim.keys()) == set(jnt_ranges_mjc.keys()):
            logger.debug("Joint names are not matching")
            raise('\nJoints in OpenSim and MuJoCo are different for the evaluation\n')
        
        else:
            
            # DO NOT RAISE ERROR BUT WARNING, ASSUME THERE IS NO DIFFERENCE (RISK)
            for jnt in jnt_ranges_osim.keys():
                # count for decimal round 
                if not ((np.abs(jnt_ranges_osim[jnt] - jnt_ranges_mjc[jnt])) < 0.005).all():
                    logger.warning(f"Joint range at {jnt} are not matching")

            # save the geo results
            jnt_ranges_osim_file = open(self.saving_path + '/jnt_ranges_osim.pkl', 'wb')
            pickle.dump(jnt_ranges_osim, jnt_ranges_osim_file)
            jnt_ranges_osim_file.close()
            
            jnt_ranges_mjc_file = open(self.saving_path + '/jnt_ranges_mjc.pkl', 'wb')
            pickle.dump(jnt_ranges_mjc, jnt_ranges_mjc_file)
            jnt_ranges_mjc_file.close()

            logger.info("Finished with joint operation range check. Saved Results")
            
        self.endPointsPlot(end_pos_osim, end_pos_mjc, end_points_osim)
    

    def endPointsPlot(self, end_pos_osim, end_pos_mjc, end_points):
        
        # plot each marker location error in x, y, z
        logger.info("Plot each endpoint's location errors in x, y, z.")

        # mlist = ['o', '^', 's', '<', '+', 'X', '>', 'd', '1', '*']
        # clist = list(mc.TABLEAU_COLORS.keys())*10
        
        # run over the markers
        marNum = len(end_points)

        mr_rms = []  # RMS of the end point location differeOUTPUT_FOLDERnces
        
        for nm in range(marNum):

            logger.info(f"Endpoint : {end_points[nm]}")

            ## plot in a x, y, x subplot way, all markers will be included in to one subplot
            xyzPlot = plt.figure(figsize=(10, 8))
            axx = xyzPlot.add_subplot(3,1,1)
            axy = xyzPlot.add_subplot(3,1,2)
            axz = xyzPlot.add_subplot(3,1,3)
        
            # extract all the position data of this marker
            osim_mr = []
            for item in end_pos_osim:
                osim_mr.append(item[nm])
            # convert to array
            osim_mr = np.array(osim_mr)
                            
            mjc_mr = []
            for item in end_pos_mjc:
                mjc_mr.append(item[nm, :])
            # convert to array
            mjc_mr = np.array(mjc_mr)
            
            mr_rms.append((np.sqrt((osim_mr - [1, 1, -1]*mjc_mr[:, [0, 2, 1]])**2)).sum(axis=1)) 
            
            x = np.linspace(1, len(mjc_mr[:, 0]), len(mjc_mr[:, 0]))
        
            # opensim and mujoco have different coordinate, here we use the opensim
            # one for plotting, need to plot mujoco data in openSim coordindate
            if nm == 0:
                axx.scatter(1, osim_mr[0, 0]*100, marker='o', c='r', linewidths=1.5, edgecolors='k', label='Osim')
                axx.scatter(1, mjc_mr[0, 0]*100, marker='^', c='b', linewidths=1.5, edgecolors='k', label='Mjc_Cvt1')
                # axx.set_title('Forward Kinematics Check')

            axx.scatter(1, osim_mr[0, 0]*100, marker='o', c='b', label= end_points[nm])
            axx.scatter(x, osim_mr[:, 0]*100, s=150, marker='o', c='r', linewidths=1.5, edgecolors='k', alpha=1)
            axx.scatter(x, mjc_mr[:, 0]*100, s=150, marker='^', c='b', linewidths=1.5, edgecolors='k', alpha=1)
            axx.set_ylabel('X axis - cm')
            axx.legend()
            
            axy.scatter(x, osim_mr[:, 1]*100, s=150, marker='o', c='r', linewidths=1.5, edgecolors='k', alpha=1)
            axy.scatter(x, mjc_mr[:, 2]*100, s=150, marker='^', c='b', linewidths=1.5, edgecolors='k', alpha=1)
            axy.set_ylabel('Y axis - cm')
            
            axz.scatter(x, osim_mr[:, 2]*100, s=150, marker='o', c='r', linewidths=1.5, edgecolors='k', alpha=1)
            axz.scatter(x, -mjc_mr[:, 1]*100, s=150, marker='^', c='b', linewidths=1.5, edgecolors='k', alpha=1)
            axz.set_ylabel('Z axis - cm')
            axz.set_xlabel('Mesh Ponits')
            
            plt.suptitle('Forward Kinematics Check')
            
            xyzPlot.savefig(self.saving_path + '/end_points/'+ end_points[nm] +'.svg', format = "svg")

            plt.close(xyzPlot)

        # save mean and std of the overall endpoint errors
        end_point_error = {}
        end_point_error['mean'] = np.mean(mr_rms)
        end_point_error['std'] = np.std(mr_rms)

        end_point_error_file = open(self.saving_path + '/end_points/end_point_error.pkl', 'wb')
        pickle.dump(end_point_error, end_point_error_file)
        end_point_error_file.close()


        ## plot error bar of the end points at different testing points
        subPlotMarkerNum = 10

        logger.info(f"Generate overall error Bar plot")

        figname2 = self.saving_path + '/end_points/overall_comp_error_bar.svg'
        barPlot = plt.figure(figsize=(10, 8))
        numSubPlot = int(np.ceil(marNum/subPlotMarkerNum))
        resSubPlotMar = marNum % subPlotMarkerNum

        for subPlot in range(numSubPlot):
            axx = barPlot.add_subplot(numSubPlot, 1, subPlot+1)
            if subPlot == numSubPlot - 1:
                if not resSubPlotMar == 0:  # if resSubPlotMarker is not 0.
                    plt.boxplot(mr_rms[subPlot*subPlotMarkerNum:])
                    plt.xticks(np.linspace(1, resSubPlotMar, resSubPlotMar, dtype=int), end_points[subPlot*subPlotMarkerNum:(subPlot+1)*subPlotMarkerNum])
            else:
                plt.boxplot(mr_rms[subPlot*subPlotMarkerNum:(subPlot+1)*subPlotMarkerNum])
                plt.xticks(np.linspace(1, subPlotMarkerNum, subPlotMarkerNum, dtype=int), end_points[subPlot*subPlotMarkerNum:(subPlot+1)*subPlotMarkerNum])
            # plt.xticks(rotation=90)
            if subPlot == 0:
                plt.title('RMS errors of end points at checking postures')
            plt.ylabel('m')
        barPlot.savefig(figname2, format = "svg")
        plt.close()
        
