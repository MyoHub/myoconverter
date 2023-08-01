#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Created on Thu May  5 11:16:03 2022
# @author: Huawei Wang

# PyOpenGL/MuJoCo fails in Linux if running with something else than osmesa; needs to be set before mujoco is imported
import platform
if platform.system() == "Linux":
    import os
    os.environ["MUJOCO_GL"] = "osmesa"

import argparse
from loguru import logger
from myoconverter.conversion_steps.O2MSteps import O2MSteps
import pickle
import os

def O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs):
    """
    :param osim_file: Path to the OpenSim OSIM model file
    :param geometry_folder: Path to the Geometry folder
    :param output_folder: Path to folder where converted model is saved
    :param : Selected conversion steps, any subset of [1, 2, 3]
    :param kwargs: convert_step, muscle_list, osim_data_overwrite, convert, 
        validation, generate_pdf, speedy, add_ground_geom, 
        treat_as_normal_path_point
    :return:
    """

    # based on updated configure options, run the pipeline
    convert_steps = kwargs.get("convert_steps", [1, 2, 3])
    muscle_list = kwargs.get("muscle_list", None)
    osim_data_overwrite = kwargs.get("osim_data_overwrite", False)
    conversion = kwargs.get("conversion", True)
    validation = kwargs.get("validation", True)
    generate_pdf = kwargs.get("generate_pdf", False)
    speedy = kwargs.get("speedy", False)
    add_ground_geom = kwargs.get("add_ground_geom", False)
    treat_as_normal_path_point = kwargs.get("treat_as_normal_path_point", False)

    # set logging
    MODEL_NAME = os.path.split(osim_file)[1][:-5]
    OUTPUT_LOG_FILE = os.path.join(output_folder, f"{MODEL_NAME}_conversion.log")

    # If there is an existing log file, remove it
    if os.path.exists(OUTPUT_LOG_FILE):
        os.remove(OUTPUT_LOG_FILE)

    # Set the log file
    logger.add(OUTPUT_LOG_FILE)

    logger.info(f"Start the conversion pipeline for : {MODEL_NAME}")

    # coordinate configure options
    osim_data_overwrite = coordinate_kwargs(osim_file, output_folder, convert_steps,\
                                   osim_data_overwrite, conversion, validation, speedy)

    O2MSteps_inst = O2MSteps(osim_file, geometry_folder, output_folder,\
                 convert_steps = convert_steps, muscle_list = muscle_list,\
                 osim_data_overwrite = osim_data_overwrite, convert = conversion,\
                 validation = validation, generate_pdf = generate_pdf, speedy = speedy,\
                 add_ground_geom = add_ground_geom, treat_as_normal_path_point = treat_as_normal_path_point)
    
    O2MSteps_inst.PipelineExecution()

    logger.remove()

def coordinate_kwargs(osim_file, output_folder, convert_steps,\
                                   osim_data_overwrite, conversion, validation, speedy):
    """
    Some config flags may conflicting with each other, this coordinate step is to manage them.
    """

    # write down the configure coordinate info
    logger.info("Coordinate the input configurations, remove conflicts.")

    # if conversion is not selected, then no need to regenerate osim dataset
    # Give back this control to users ...
    # if not conversion:
    #     osim_data_overwrite = False
    #     logger.info("   Optimization process was not selected, set osim_data_overwrite to FALSE.")

    # then check the last saved configure files in the second and third steps
    if 2 in convert_steps:

        logger.info("   Checking configurations for step 2 conversion.")

        if os.path.isfile(output_folder + '/Step2_muscleKinematics/config.pkl'):
            with open(output_folder + '/Step2_muscleKinematics/config.pkl', 'rb') as old_configure_file:
                old_config = pickle.load(old_configure_file)

            # if speedy flag changes, overwriting is always needed for optimization
            if old_config["speedy"] != speedy:
                if conversion:
                    osim_data_overwrite = True

                    logger.info("       Speedy flag is different from last run, set osim_data_overwrite to TRUE.")

        else:
            osim_data_overwrite = True
            logger.info("       Configure of previous run does not exist, set osim_data_overwrite to TRUE.")

        logger.info("       All good now.")

        # save kwargs
        kwargs = {}
        kwargs["convert_steps"] = convert_steps
        kwargs["osim_data_overwrite"] = osim_data_overwrite
        kwargs["conversion"] = conversion
        kwargs["speedy"] = speedy

        # save the update kwargs to config.pkl in step 2 folder
        os.makedirs(output_folder + '/Step2_muscleKinematics', exist_ok = True)
        config_save = open(output_folder + '/Step2_muscleKinematics/config.pkl', 'wb')
        pickle.dump(kwargs, config_save)
        config_save.close()

    if 3 in convert_steps:

        logger.info("   Checking configurations for step 3 conversion.")

        if os.path.isfile(output_folder + '/Step3_muscleKinetics/config.pkl'):
            with open(output_folder + '/Step3_muscleKinetics/config.pkl', 'rb') as old_configure_file:
                old_config = pickle.load(old_configure_file)

            # if speedy flag changes, overwriting is always needed for optimization
            if old_config["speedy"] != speedy:
                if conversion:
                    osim_data_overwrite = True

                    logger.info("       Speedy flag is different from last run, set osim_data_overwrite to TRUE.")

        else:
            osim_data_overwrite = True
            logger.info("       Configure of previous run does not exist, set osim_data_overwrite to TRUE.")

        logger.info("       All good now.")

        # save kwargs
        kwargs = {}
        kwargs["convert_steps"] = convert_steps
        kwargs["osim_data_overwrite"] = osim_data_overwrite
        kwargs["conversion"] = conversion
        kwargs["speedy"] = speedy

        # save the update kwargs to config.pkl in step 3 folder
        os.makedirs(output_folder + '/Step3_muscleKinetics', exist_ok = True)
        config_save = open(output_folder + '/Step3_muscleKinetics/config.pkl', 'bw')
        pickle.dump(kwargs, config_save)
        config_save.close()
    
    return osim_data_overwrite

    
                
if __name__ == "__main__":

    argparser = argparse.ArgumentParser(description='Convert an OpenSim model into a MuJoCo model with accurate muscle kinetics.'
                                                 'Only Works with OpenSim v4 models.')
    argparser.add_argument('osim_file', type=str,
                           help='Path to an OpenSim model OSIM file')
    argparser.add_argument('geometry_folder', type=str,
                           help="Path to the Geometry folder (by default uses folder of given OpenSim file)")
    argparser.add_argument('output_folder', type=str,
                           help="Path to an output folder. The converted model will be saved here.")
    
    argparser.add_argument('--convert_steps', type=list, default= [1, 2, 3],
                           help="Selected conversion steps, could be any subset of [1, 2, 3] based on the needs")
    argparser.add_argument('--muscle_list', type=list, default=None,
                           help='Selected muscles for the conversion steps')
    argparser.add_argument('--osim_data_overwrite', default=False,
                           help='If ture, overwrite extracted Osim model state files')
    argparser.add_argument('--conversion', default=True, 
                           help="If true, perform the conversion functions of selected steps")
    argparser.add_argument('--validation', default=True, 
                           help="If true, perform the validation functions of selected steps")
    argparser.add_argument('--speedy', default=False,
                           help="If true, reduce the number of checking notes in optimization steps")
    argparser.add_argument('--generate_pdf', default=False,
                           help="If true, generate a pdf report of the validation results")
    argparser.add_argument('--add_ground_geom', default=False,
                           help="If true, a geom (of type plane) is added to the MuJoCo model as ground")
    argparser.add_argument('--treat_as_normal_path_point', default=False,
                           help="If true, MovingPathPoints and ConditionalPathPoints will be treated as normal "
                                "PathPoints")
    args = argparser.parse_args()

    # Do the pipeline
    O2MPipeline(**vars(args))