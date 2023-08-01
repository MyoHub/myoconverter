import os
import glob
from fpdf import Template
from loguru import logger
import pickle

def generate_pdf(mjc_vlt1_path:str, mjc_vlt2_path:str, mjc_vlt3_path:str, model_name:str, save_path:str) -> Template:
    """
    Generate pdf report with all the validation plots
    """

    # define a template
    elements = [
        {'name': 'main_title', 'type': 'T', 'x1': 20.0, 'y1': 17.0, 'x2': 280.0, 'y2': 28.0,\
          'font': 'helvetica', 'size': 28, 'bold': 1, 'italic': 0, 'text': '', 'multiline': False,},

        {'name': 'plant_text_title', 'type': 'T', 'x1': 20.0, 'y1': 35.0, 'x2': 280.0, 'y2': 45.0,\
          'font': 'helvetica', 'size': 16, 'bold': 0, 'italic': 0, 'text': '', 'multiline': True,},

        {'name': 'plant_text1', 'type': 'T', 'x1': 20.0, 'y1': 30.0, 'x2': 280.0, 'y2': 40.0,\
          'font': 'helvetica', 'size': 14, 'bold': 0, 'italic': 0, 'text': '', 'multiline': True,},

        {'name': 'plant_text0', 'type': 'T', 'x1': 20.0, 'y1': 40.0, 'x2': 280.0, 'y2': 45.0,\
          'font': 'helvetica', 'size': 14, 'bold': 1, 'italic': 1, 'text': '', 'foreground': 0x110000,\
           'multiline': True,},

        {'name': 'list_title1', 'type': 'T', 'x1': 40.0, 'y1': 60.0, 'x2': 280.0, 'y2': 70.0,\
          'font': 'helvetica', 'size': 20, 'bold': 1, 'italic': 0, 'text': '', 'multiline': False,},

        {'name': 'list_content1', 'type': 'T', 'x1': 50.0, 'y1': 70.0, 'x2': 280.0, 'y2': 80.0,\
          'font': 'helvetica', 'size': 16, 'bold': 0, 'italic': 0, 'text': '', 'multiline': True,},

        {'name': 'error_content1', 'type': 'T', 'x1': 50.0, 'y1': 90.0, 'x2': 280.0, 'y2': 100.0,\
          'font': 'helvetica', 'size': 16, 'bold': 0, 'italic': 1, 'text': '', 'foreground': 0x110000,\
          'multiline': False,},

        {'name': 'list_title2', 'type': 'T', 'x1': 40.0, 'y1': 110.0, 'x2': 280.0, 'y2': 120.0,\
          'font': 'helvetica', 'size': 20, 'bold': 1, 'italic': 0, 'text': '', 'multiline': False,},

        {'name': 'list_content2', 'type': 'T', 'x1': 50.0, 'y1': 120.0, 'x2': 280.0, 'y2': 130.0,\
          'font': 'helvetica', 'size': 16, 'bold': 0, 'italic': 0, 'text': '', 'multiline': True,},

        {'name': 'error_content2', 'type': 'T', 'x1': 50.0, 'y1': 130.0, 'x2': 280.0, 'y2': 140.0,\
          'font': 'helvetica', 'size': 16, 'bold': 0, 'italic': 1, 'text': '', 'foreground': 0x110000,\
          'multiline': False,},

        {'name': 'list_title3', 'type': 'T', 'x1': 40.0, 'y1': 150.0, 'x2': 280.0, 'y2': 160.0,\
          'font': 'helvetica', 'size': 20, 'bold': 1, 'italic': 0, 'text': '', 'multiline': False,},

        {'name': 'list_content3', 'type': 'T', 'x1': 50.0, 'y1': 160.0, 'x2': 280.0, 'y2': 170.0,\
          'font': 'helvetica', 'size': 16, 'bold': 0, 'italic': 0, 'text': '', 'multiline': True,},

        {'name': 'error_content3', 'type': 'T', 'x1': 50.0, 'y1': 180.0, 'x2': 280.0, 'y2': 190.0,\
          'font': 'helvetica', 'size': 16, 'bold': 0, 'italic': 1, 'text': '', 'foreground': 0x110000,\
          'multiline': False,},

        {'name': 'step_title', 'type': 'T', 'x1': 20.0, 'y1': 17.0, 'x2': 280.0, 'y2': 27.0,\
          'font': 'helvetica', 'size': 24, 'bold': 1, 'italic': 0, 'text': '', 'multiline': False,},

        {'name': 'plant_text2', 'type': 'T', 'x1': 20.0, 'y1': 75.0, 'x2': 280.0, 'y2': 85.0,\
          'font': 'helvetica', 'size': 14, 'bold': 0, 'italic': 0, 'text': '', 'multiline': True,},

         {'name': 'plant_text3', 'type': 'T', 'x1': 20.0, 'y1': 95.0, 'x2': 280.0, 'y2': 105.0,\
          'font': 'helvetica', 'size': 14, 'bold': 0, 'italic': 0, 'text': '', 'multiline': True,},

        {'name': 'plot_title', 'type': 'T', 'x1': 10.0, 'y1': 5.0, 'x2': 280.0, 'y2': 10.0,\
          'font': 'helvetica', 'size': 18, 'bold': 0, 'italic': 0, 'text': '', 'multiline': False,},

        {'name': 'overall_image', 'type': 'I', 'x1': 20.0, 'y1': 15.0, 'x2': 280.0, 'y2': 210.0,},

        {'name': 'xml_image', 'type': 'I', 'x1': 20.0, 'y1': 15.0, 'x2': 280.0, 'y2': 210.0,},

        {'name': 'ma_image', 'type': 'I', 'x1': 10.0, 'y1': 20.0, 'x2': 300.0, 'y2': 200.0,},

        {'name': 'mf_image', 'type': 'I', 'x1': 10.0, 'y1': 20.0, 'x2': 300.0, 'y2': 200.0,},]
    # if generate pdf report, then initilize the pdf file, Landscape A4 (297 by 210 mm) with element template
    pdf = Template(format = 'A4', elements = elements, orientation = 'L', unit = 'mm',\
                    title = 'O2M myoConverter Model Validation Report',\
                    author=' Huawei Wang & Aleksi Ikkala',\
                    keywords='Musculoskeletal Model, MuJoCo, OpenSim, Muscle Kinematics & Kinetics, Conversion pipeline') # Landscape A4 (210 by 297 mm)
    
    
    def add_main_page(pdf:Template, model_name:str, mjc_vlt1_path:str, mjc_vlt2_path:str,\
                      mjc_vlt3_path:str) -> Template:
        """ Add main page to the pdf
        """

        # The main page contains the title, a summary of how good the model is, 
        # and overview of the validation steps
        # Extract overall RMS and STD

        # First Page
        pdf.add_page()

        pdf['main_title'] = 'Validation of converted ' + model_name + ' model '

        pdf['plant_text_title'] = "The converted MJC model has been tested under the three categories below" + \
                            " and with the accuracy of:"
        
        pdf['list_title1'] = '- Step 1: XML Conversion Validation'
        pdf['list_content1'] = 'Check multi-body forward kinematics (using endpoints), approximation of custom/coupling joints' + \
                              ' & conditional/moving path points'
        
        # load the end point data from the saved files
        if os.path.isfile(mjc_vlt1_path + '/end_points/end_point_error.pkl'):
          with open(mjc_vlt1_path + '/end_points/end_point_error.pkl', "rb+") as endpoint_error_file:
              endpoint_error = pickle.load(endpoint_error_file)

          pdf['error_content1'] = 'Mean error:' + str(round(endpoint_error['mean']*100, 4)) + " cm" + \
                            '; std: ' +  str(round(endpoint_error['std']*100, 4)) + " cm"

        pdf['list_title2'] = '- Step 2: Muscle Kinematics Validation'
        pdf['list_content2'] = 'Check muscle moment arms as indication how muscle wrap over joints'

        # load the moment arm error data from the saved files
        if os.path.isfile(mjc_vlt2_path + '/overall_comp_momentarms.pkl'):
          with open(mjc_vlt2_path + '/overall_comp_momentarms.pkl', "rb+") as momentarm_error_file:
            momentarm_error = pickle.load(momentarm_error_file)

          pdf['error_content2'] = 'Mean error:' + str(round(momentarm_error['cost_opt_mat_mean']*100, 4)) + " cm" + \
                            '; std: ' +  str(round(momentarm_error['cost_opt_mat_std']*100, 4)) + " cm" 

        pdf['list_title3'] = '- Step 3: Muscle Kinetic Validation'
        pdf['list_content3'] = 'Check muscle force-length relationship as indication of how similar of them in generating forces'
        # load the muscle force error data from the saved files
        if os.path.isfile(mjc_vlt3_path + '/overall_comp_muscleforces.pkl'):
          with open(mjc_vlt3_path + '/overall_comp_muscleforces.pkl', "rb+") as muscleforce_error_file:
            muscleforce_error = pickle.load(muscleforce_error_file)

          pdf['error_content3'] = 'Mean error:' + str(round(muscleforce_error['rms_opt_mean'], 4)) + " Fmax" + \
                            '; std: ' +  str(round(muscleforce_error['rms_opt_std'], 4)) + " Fmax"
        
        return pdf


    def add_step1_page(pdf:Template) -> Template:

        pdf.add_page()

        pdf['step_title'] = 'Step 1: xml Conversion Validation'

        pdf['plant_text1'] = 'Randomly pose the model with 10 confgurations within the joint limits. ' + \
            'In each posture, the endpoints(markers) global locations of Osim and Mjc models are extracted and compared. ' + \
            'Box plot of their mean-std errros are plotted together. Individual endpoint differences of these 10 postures' + \
            ' are also ploted in the VLT folder, but not included inside this report.'
        
        pdf['plant_text2'] = 'Besides the endpoint check, the approximation of customer joints, coupling joints,' + \
            ' conditional/moving path points are plotted and attached. In these plots, blue dots/lines represent ' + \
            ' their setup in the OpenSim model. Yellow dots/lines represent the approximations in the MuJoCo model.'

        return pdf

    def add_step1_plots(pdf:Template, mjc_vlt1_path:str) -> Template:
      
        logger.info("   Adding vlt1 results.")

        # Scan the end_points folder to find all .svg files and add the overall comparison to the pdf report
        vlt1_plot_list = [os.path.split(f)[1][0:-4] for f in glob.glob(mjc_vlt1_path + "/end_points/*.svg")]

        if len(vlt1_plot_list) == 1:  # only empty overall_comp plot generated, no end point plots
            pdf.add_page()
            pdf['step_title'] = "NO End Points found in the model."
            
        else:
            # first add the overall comparsion plots
            for vlt1_name in vlt1_plot_list:
                if "overall_comp" in vlt1_name:
                    pdf.add_page()
                    pdf['plot_title'] = "End points"
                    pdf['overall_image'] = mjc_vlt1_path + '/end_points/' + vlt1_name + '.svg'

        # Scan the coordinate_coupler_constraints folder to find all .svg files and add the overall comparison to the pdf report
        if os.path.isdir(mjc_vlt1_path + "/coordinate_coupler_constraints"):

            vlt1_plot_list = [os.path.split(f)[1][0:-4] for f in glob.glob(mjc_vlt1_path + "/coordinate_coupler_constraints/*.svg")]
            # add all plots
            for vlt1_name in vlt1_plot_list:
                pdf.add_page()
                pdf['plot_title'] = "Approximation of coupling joints"
                pdf['xml_image'] = mjc_vlt1_path + '/coordinate_coupler_constraints/' + vlt1_name + '.svg'
        
        # Scan the custom joints folder to find all .svg files and add the overall comparison to the pdf report
        if os.path.isdir(mjc_vlt1_path + "/custom_joints"):

            vlt1_plot_list = [os.path.split(f)[1][0:-4] for f in glob.glob(mjc_vlt1_path + "/custom_joints/*.svg")]
            # add all plots
            for vlt1_name in vlt1_plot_list:
                pdf.add_page()
                pdf['plot_title'] = "Approximation of custom joints"
                pdf['xml_image'] = mjc_vlt1_path + '/custom_joints/' + vlt1_name + '.svg'

        # Scan the moving path folder to find all .svg files and add the overall comparison to the pdf report
        if os.path.isdir(mjc_vlt1_path + "/moving_path_points"):

            vlt1_plot_list = [os.path.split(f)[1][0:-4] for f in glob.glob(mjc_vlt1_path + "/moving_path_points/*.svg")]
            # add all plots
            for vlt1_name in vlt1_plot_list:
                pdf.add_page()
                pdf['plot_title'] = "Approximation of moving path points"
                pdf['xml_image'] = mjc_vlt1_path + '/moving_path_points/' + vlt1_name + '.svg'

        # Scan the moving path folder to find all .svg files and add the overall comparison to the pdf report
        if os.path.isdir(mjc_vlt1_path + "/conditional_path_points"):

            vlt1_plot_list = [os.path.split(f)[1][0:-4] for f in glob.glob(mjc_vlt1_path + "/conditional_path_points/*.svg")]
            # add all plots
            for vlt1_name in vlt1_plot_list:
                pdf.add_page()
                pdf['plot_title'] = "Approximation of conditional path points"
                pdf['xml_image'] = mjc_vlt1_path + '/conditional_path_points/' + vlt1_name + '.svg'

        return pdf

    def add_step2_page(pdf:Template) -> Template:

        pdf.add_page()

        pdf['step_title'] = 'Step 2: Muscle Kinematics Validation'

        pdf['plant_text1'] = 'Moment arm of each muscle at each joint are compared between Osim and converted MJC model. ' + \
            'A overall heatmap is included to indicate the overall moment arm errors before and after optimization. ' + \
            'Then detail moment arm curves are plotted for comparison. ' + \
            'For the muscles that wrap over multiple joints, moment arms with respect to one joint maybe affected by several other joints. ' + \
            'In this case, several mesh points were check of these affecting joints, when plotting moment arms at one joint. ' + \
            'This is why there are multiple lines (with different grey levels) plotted for one muscle on one joint. '
        
        pdf['plant_text3'] = 'How to interpret the plot: \n' + \
            'Global title indicate the muscle and joints that affecting the moment arms in the plots. ' + \
            'X axis indicate the joint that moment arms were extracted. ' + \
            'Grey level of the lines indicate the mesh postures of other relevant joints (in the global tile, but not the x axis)'

        return pdf
                
    def add_step2_plots(pdf:Template, mjc_vlt2_path:str) -> Template:

        logger.info("   Adding vlt2 results.")
        # Scan the result folder to find all the <muscle_name>.png files and add all of them to the pdf report
        vlt2_plot_list = [os.path.split(f)[1][0:-4] for f in glob.glob(mjc_vlt2_path + "/moment_arms/*.svg")]

        # first add the overall comparsion plots
        for vlt2_name in vlt2_plot_list:
            if "overall_comp" in vlt2_name:
                pdf.add_page()
                pdf['plot_title'] = "Overall comparison of muscle moment arms before/after optimization"
                pdf['ma_image'] = mjc_vlt2_path + '/moment_arms/' + vlt2_name + '.svg'

        for vlt2_name in vlt2_plot_list:
            if not "overall_comp" in vlt2_name:
                pdf.add_page()
                pdf['plot_title'] = "Muscle specific moment arm comparison before/after optimization"
                pdf['ma_image'] = mjc_vlt2_path + '/moment_arms/' + vlt2_name + '.svg'

        return pdf


    def add_step3_page(pdf:Template) -> Template:

        pdf.add_page()

        pdf['step_title'] = 'Step 3: Muscle Kinetic Validation'

        pdf['plant_text1'] = 'Muscle force-length property are compared between Osim and Mjc models. ' + \
            'This force-length property only depends on muscle-fiber-tendon unit lengths. ' + \
            'We made it isolated with the moment arm, so that the change in moment arms will not affect the muscle force properties. ' + \
            'The muscle-fiber-tendon unit lengths were roughly even extracted (from shorest to longest) with all possible body postures. ' + \
            'A bar plot of the froce errors of all muscle before and after optimization is included. ' + \
            'Then the detail force-length curve comparsion plot of each muscle is included. '
        
        pdf['plant_text3'] = 'How to interpret the plot: \n' + \
            'Global title indicate the muscle name. ' + \
            'X axis indicate the muscle-fiber-tendon unit length. ' + \
            'Y axis is the muscle force (unnormalized)'

        return pdf


    def add_step3_plots(pdf:Template, mjc_vlt3_path:str) -> Template:

        logger.info("   Adding vlt3 results.")

        # Scan the result folder to find all the <muscle_name>.png files and add all of them to the pdf report
        vlt3_plot_list = [os.path.split(f)[1][0:-4] for f in glob.glob(mjc_vlt3_path + "/muscle_forces/*.svg")]

        # first add the overall comparsion plots
        for vlt3_name in vlt3_plot_list:
            if "overall_comp" in vlt3_name:
                pdf.add_page()
                pdf['plot_title'] = "Overall comparison of muscle force-length relationship before/after optimization"
                pdf['ma_image'] = mjc_vlt3_path + '/muscle_forces/' + vlt3_name + '.svg'

        for vlt3_name in vlt3_plot_list:
            if not "overall_comp" in vlt3_name:
                pdf.add_page()
                pdf['plot_title'] = "Muscle specific force comparison before/after optimization"
                pdf['ma_image'] = mjc_vlt3_path + '/muscle_forces/' + vlt3_name + '.svg'

        return pdf
    
    pdf = add_main_page(pdf, model_name, mjc_vlt1_path, mjc_vlt2_path, mjc_vlt3_path)

    pdf = add_step1_page(pdf)
    pdf = add_step1_plots(pdf, mjc_vlt1_path)

    pdf = add_step2_page(pdf)
    pdf = add_step2_plots(pdf, mjc_vlt2_path)

    pdf = add_step3_page(pdf)
    pdf = add_step3_plots(pdf, mjc_vlt3_path)

    # # generate pdf
    # pdf.render(save_path + '/' + model_name + '.pdf')
    
    return pdf 