from loguru import logger
from myoconverter.conversion_steps.O2MStep1 import BasicModelConvert
from myoconverter.conversion_steps.O2MStep2 import MomentArmOpt
from myoconverter.conversion_steps.O2MStep3 import MuscleForceOpt
from myoconverter.utils.generate_pdf import generate_pdf
import os
from fpdf import Template

class O2MSteps:
    """
    Class to coordinate all conversion steps
    """

    def __init__(self, osim_model_file, geom_folder, mjc_target_folder,\
                 convert_steps = [1, 2, 3], muscle_list = None,\
                 osim_data_overwrite = False, convert = False,\
                 validation = False, generate_pdf = False, speedy = False,\
                 add_ground_geom = False,\
                 treat_as_normal_path_point = False):
            
        """
        Parameters:
        osim_model_file: string
            The OpenSim model file. Only works with Osim 4.0+. Osim 3.0 models
            can be saved into Osim 4.0+ by opening and saving in OpenSim 4.0+ software.

        geo_folder: string
            The geometry folder of the given Osim model.

        mjc_target_folder: string
            The target folder where the converted mjc model and validation results
            will be saved.

        convert_steps: list of integer numbers [1, 2, 3]
            Corresponding to the three major steps, can be customized.
            For instance, '[1]' refers to the 'general model conversion step';
            '[1, 2]' refers to the 'general model conversion' & 'moment arm optimzation step';
            '[3]' refers to the 'muscle force optimzation step', but will require that
            the [1, 2] has been ran before (model are saved).

        muscle_list: list of string
            If not NONE, the list of selected muscles that will be processed only.

        osim_data_overwrite: boolean
            If True, overwrite the extracted data of Osim states.

        convert: boolean
            If True, run convert processed of the selected steps.

        validation: boolean
            If True, perform validation (generate plots) for the selected steps.

        generate_pdf: boolean
            If True, generate the pdf report for the converted model 
            This generation only reads the generated comparison plots from the validation steps.
            Therefore, suggest to run after all the validation steps were preformed.

        speedy: boolean
            If True, the optimization process will select a low number of instances, 
            checking nodes, as well iterations to speed up the process. This may result less
            optimal results, however for complex models, this is suggested. Then regular optimization
            process (slower) can be done on selected muscles, if detected issues. 

        add_ground_geom: boolean
            If True, add ground to the converted model.

        treat_as_normal_path_point: boolean
            If True, treat all moving and conditional points as normal fix points.        

        """

        # # set logging
        # MODEL_NAME = os.path.split(osim_model_file)[1][:-5]
        # OUTPUT_LOG_FILE = os.path.join(mjc_target_folder, f"{MODEL_NAME}-conversion.log")

        # # If there is an existing log file, remove it
        # if os.path.exists(OUTPUT_LOG_FILE):
        #     os.remove(OUTPUT_LOG_FILE)

        # # Set the log file
        # logger.add(OUTPUT_LOG_FILE)

        # Output file infos
        logger.info(f"Given OpenSim model file: {osim_model_file}")
        logger.info(f"Reading mesh files from folder {geom_folder}")
        logger.info(f"Converted model will be saved in folder {mjc_target_folder}")

        # transfer variables
        self.osim_model_file = osim_model_file
        self.geom_folder = geom_folder
        self.mjc_target_folder = mjc_target_folder
        self.convert_steps = convert_steps
        self.muscle_list = muscle_list
        self.osim_data_overwrite = osim_data_overwrite  
        self.convert = convert
        self.validation = validation  
        self.generate_pdf = generate_pdf
        self.speedy = speedy
        self.add_ground_geom = add_ground_geom
        self.treat_as_normal_path_point = treat_as_normal_path_point

    def PipelineExecution(self):
    	
        # create the mjc_target_folder
        os.makedirs(self.mjc_target_folder, exist_ok = True)
	    
	    ##############################################################################
	    # STEP 1: GEOMETRY CONVERTING
	    # convert the geomerties
	    
	    # check if the first step is required
        if 1 in self.convert_steps:

	        # create the first step folder if not exist
            path_step1 = self.mjc_target_folder + '/Step1_xmlConvert'
            os.makedirs(path_step1, exist_ok = True)
    		
    	    # load the first step converting class
            Step1 = BasicModelConvert(self.osim_model_file, self.geom_folder, path_step1,\
                                      add_ground_geom = self.add_ground_geom,\
                                      treat_as_normal_path_point = self.treat_as_normal_path_point)
    		
            if self.convert: # General model conversion, generate xxxx_cvt1.xml model
                logger.info("Started step 1 conversion - cvt1.")
                mjcModel_Cvt1_path = Step1.cvt1_ModelConvert()
                logger.info("Finished step 1 conversion - cvt1.") 

            else:
                logger.info("convert is set to False, skipped step 1 conversion - cvt1.")
                        
            if self.validation:  # validation of the general model conversion, forward kinematics check
                logger.info("Started step 1 validation - val1.")
                if self.convert:
                    Step1.vlt1_forwardKinematicsValidation(speedy = self.speedy)
                else:
                    mjcModel_Cvt1_path = self.mjc_target_folder + '/' +\
                            os.path.split(self.osim_model_file)[1][:-5] +\
                            '_cvt1.xml'
                                
                    Step1.vlt1_forwardKinematicsValidation(mjc_model_path = mjcModel_Cvt1_path, speedy = self.speedy)

                logger.info("Finished step 1 validation - val1.")
                
            else:
                logger.info("validation is set to False, skipped step 1 validation - vlt1.")

        else:
            logger.info("[1] is not selected, skipped step 1 process.")

	    ##############################################################################

        if 2 in self.convert_steps:
	    ###############################################################################
    	    # STEP 2: MOMENT ARM OPTIMIZATION
    	    
            # The cvt1 model name path(after the 1st step general conversion) from default setting.
            mjcModel_Cvt1_path = self.mjc_target_folder + '/' +\
                            os.path.split(self.osim_model_file)[1][:-5] +\
                            '_cvt1.xml'
            
            # The cvt2 model name path(after the 2nd step convert) from default setting.
            mjcModel_Cvt2_path = self.mjc_target_folder + '/' +\
                            os.path.split(self.osim_model_file)[1][:-5] +\
                            '_cvt2.xml'
            
            # First check if the cvt2 model is already exist. 
            # If so, then directly optimize on top of it.
            # Otherwise, optimize from cvt1 model and generate cvt2 model at the end.
            # If cvt1 model does not exist either, then raise an error.
            if os.path.exists(mjcModel_Cvt2_path):
                logger.info("xxx_cvt2 model already exist, process from it")
                cvt2_model_path = mjcModel_Cvt2_path

            else:
                logger.info("xxx_cvt2 model does not exist, start from xxx_cvt1")

                if os.path.exists(mjcModel_Cvt1_path):
                    cvt2_model_path = mjcModel_Cvt1_path

                else:
                    logger.debug("xxx_cvt1 model file does not exist either, stopped!")
                    logger.debug("Please run [1] first to get the xxx_cvt1 model")
                    raise('xxx_cvt1 model file does not exist, cannot process following' + 
                        'steps .. \n')
    	    
    		# step 2 saving path
            path_step2 = self.mjc_target_folder + '/Step2_muscleKinematics'
            os.makedirs(path_step2, exist_ok = True)
    		
            # apply moment arm convert
            MA_Opt = MomentArmOpt(cvt2_model_path, self.osim_model_file, path_step2,\
                                  muscle_list = self.muscle_list, 
                                  osim_data_overwrite = self.osim_data_overwrite,\
    			                  speedy = self.speedy)
    		    
            if self.convert:
                # optimize the side sites to get better moment arm fits
                logger.info("Started step 2 conversion - cvt2.")
                mjcModel_Cvt2_path = MA_Opt.optMomentArms()
                logger.info("Finished step 2 conversion - cvt2.")
            else:
                logger.info("convert is set to False, skipped step 2 conversion - cvt2.")
    		
            if self.validation:
                # then validate (plot) the convert results
                logger.info("Started step 2 validation - val2.")
                MA_Opt.compMomentArmResults()
                logger.info("Finished step 2 validation - val2.")
            else:
                logger.info("validation is set to False, skipped step 2 validation - val2.")
    		
        ###############################################################################
        
        if 3 in self.convert_steps:
 	    ###############################################################################
 	    # STEP 3: MUSCULE FORCE OPTIMIZATION
  		    
            # The cvt2 model name path(after the 2nd step convert) from default setting.
            mjcModel_Cvt2_path = self.mjc_target_folder + '/' +\
                            os.path.split(self.osim_model_file)[1][:-5] +\
                            '_cvt2.xml'
            
            # The cvt3 model name path(after the 3rd step convert) from default setting.
            mjcModel_Cvt3_path = self.mjc_target_folder + '/' +\
                            os.path.split(self.osim_model_file)[1][:-5] +\
                            '_cvt3.xml'
            
            # First check if the cvt2 model is already exist. 
            # If so, then directly optimize on top of it.
            # Otherwise, optimize from cvt1 model and generate cvt2 model at the end.
            # If cvt1 model does not exist either, then raise an error.
            if os.path.exists(mjcModel_Cvt3_path):
                logger.info("xxx_cvt3 model already exist, process from it")
                cvt3_model_path = mjcModel_Cvt3_path

            else:
                logger.info("xxx_cvt3 model does not exist, start from xxx_cvt2")

                if os.path.exists(mjcModel_Cvt2_path):
                    cvt3_model_path = mjcModel_Cvt2_path

                else:
                    logger.debug("xxx_cvt2 model file does not exist either, stopped!")
                    logger.debug("Please run [2] first to get the xxx_cvt2 model")
                    raise('xxx_cvt2 model file does not exist, cannot process following' + 
                        'steps .. \n')
    		
    		# step 3 saving path
            path_step3 = self.mjc_target_folder + '/Step3_muscleKinetics'
            os.makedirs(path_step3, exist_ok = True)
    
    		# create an instance of the muscle force optimization
            musForceOpt = MuscleForceOpt(cvt3_model_path, self.osim_model_file, path_step3,\
                                         muscle_list = self.muscle_list,\
                                         osim_data_overwrite = self.osim_data_overwrite,\
    			                         speedy = self.speedy)

            if self.convert:
                # optimize muscle parameters and save optimized model as cvt3
                logger.info("Started step 3 conversion - cvt3.")
                mjcModel_Cvt3_path = musForceOpt.optMuscleForce()
                logger.info("Finished step 3 conversion - cvt3.")
            else:
                logger.info("Skipped step 3 conversion - cvt3.")
    
            if self.validation:
                # validate the muscle forces
                logger.info("Started step 3 validation - val3.")
                musForceOpt.compMuscleForceResults()
                logger.info("Finished step 3 validation - val3.")
            else:
                logger.info("Skipped step 3 validation - val3.")

	    # ###############################################################################
	    
        # if pdf report required, then generate it from the plots that generated in the
        # converted folder.
        ###############################################################################
        if self.generate_pdf:

            logger.info("Generating pdf report based on the validation results.")
            logger.info("   This may take a while, suggest to do this after all validation steps...")

            mjcModel_vlt1_path = self.mjc_target_folder + "/Step1_xmlConvert"
            mjcModel_vlt2_path = self.mjc_target_folder + "/Step2_muscleKinematics"
            mjcModel_vlt3_path = self.mjc_target_folder + "/Step3_muscleKinetics"
            model_name = os.path.split(self.osim_model_file)[1][:-5]

            pdf = generate_pdf(mjcModel_vlt1_path, mjcModel_vlt2_path, mjcModel_vlt3_path, model_name, self.mjc_target_folder)

            # generate pdf
            pdf.render(self.mjc_target_folder + '/' + model_name + '.pdf')

            logger