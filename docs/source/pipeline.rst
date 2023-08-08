Conversion Pipeline
===================

The conversion pipeline contains three major conversion steps (cvt#) as well as three validation steps (vlt#). 

In the first step an OpenSim XML file is converted into a MuJoCo XML file. As OpenSim and MuJoCo model specifications do not have unique one-to-one mappings, the produced MuJoCo model is only an approximation of the original OpenSim model. The quality of this approximation is improved in the second and third steps, where the muscle properties of the converted MuJoCo model are further optimized to better match with the original OpenSim model.

- :ref:`XML Conversion`

   - cvt1
      In the first step, an OpenSim model (.osim file) is converted into a MuJoCo model XML file. This conversion copies the fundamental elements of musculoskeletal models, such as body segments, joints, muscles, muscle paths, wrapping objects, wrapping paths, from the OpenSim file into a MuJoCo file as faithfully as possible.
   
   - vlt1
      We expect the first conversion step to provide a relatively accurate multi-body kinematic tree. Therefore, we use a validation step to check the match of body kinematics between the converted MuJoCo model and the reference OpenSim model.

- :ref:`Muscle Kinematics Optimization`

   - cvt2
      The second step adjusts the muscle kinematic properties (moment arms).
     
   - vlt2
      The validation of step 2 plots the comparison figures of muscle moment arm curves for each muscle.
      
- :ref:`Muscle Kinetics Optimization`

   - cvt3
      The third step adjusts the muscle kinetic properties (force-length relationship).
      
   - vlt3
      The validation of step 3 plots the comparison figures of muscle force-length curves for each muscle. 
      

It is highly recommended to use the main access point of the pipeline, which will perform all three steps. See the API of this access point in :py:mod:`myoconverter.O2MPipeline`


Below you can find an example of how to use the O2MPipeline.

.. code-block:: Python
   
   from myoconverter.O2MPipeline import O2MPipeline

   # General configure
   kwargs = {}  # define kwargs inputs
   kwargs['convert_steps'] = [1, 2, 3]             # All three steps selected
   kwargs['muscle_list'] = None                    # No specific muscle selected, optimize all of them
   kwargs['osim_data_overwrite'] = True            # Overwrite the Osim model state files
   kwargs['conversion'] = True                     # Yes, perform 'Cvt#' process
   kwargs['validation'] = True                     # Yes, perform 'Vlt#' process
   kwargs['generate_pdf'] = False                  # Do not generate validation pdf report
   kwargs['speedy'] = False                        # Do not reduce the checking notes to increase speed
   kwargs['add_ground_geom'] = True                # Add ground to the model
   kwargs['treat_as_normal_path_point'] = False    # Use original constraints to represent moving and conditional path points
   
   # Osim model info & target saving folder
   osim_file = './models/osim/Leg6Dof9Musc/leg6dof9musc.osim'
   geometry_folder = './models/osim/Geometry'
   output_folder = './models/converted/Leg6Dof9Musc'
   
   # Run pipeline
   O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

Meaning of the kwargs:

- `osim_file` (a string pointing to an .osim file) to a MuJoCo XML file and outputs the model into folder `output_folder`. The `**kwargs` may contain the optional parameters as listed below:
- `convert_steps` : Selected conversion steps, could be any subset of `[1, 2, 3]` based on the needs, Default = `[1, 2, 3]`
- `muscle_list` : Selected specific muscles for the conversion and validation steps, Default = None
- `osim_data_overwrite` : If true, overwrite extracted Osim model state files, Default = False [overwrite is needed, if Osim model has changed]
- `conversion` : If true, perform the conversion functions of selected steps, Default = True
- `validation` : If true, perform the validation functions of selected steps, Default = True
- `speedy` : If true, reduce the number of checking notes in optimization steps to increase speed, Default = False
- `generate_pdf` : If true, generate a pdf report of the validation results, Default = False
- `add_ground_geom` : If true, a geom (of type plane) is added to the MuJoCo model as ground, Default = False
- `treat_as_normal_path_point` : If true, MovingPathPoints and ConditionalPathPoints will be treated as normal PathPoints, Default = False
   
Outcomes of the pipeline are saved in the output folder. Four folders and a few files will be generated depending on the choice of argument options. All possible outcomes are listed below:

   - `Geometry` folder:
      Contains the .stl mesh files for the model geometries, which are copied or converted from original geometry files.

   - `Step1_xmlConvert` folder:
      This folder contains the outcomes of the first conversion and validation step. 
      Specifically, the original OpenSim and converted MuJoCo model's joint list and their ranges are stored as .pkl files.
      The endpoints kinematics check comparison plots are plotted and saved as .svg files.
      The plots of approximation of custom joints, moving, and conditional path points are generated to indicate how good the approximation are.

   - `Step2_MuscleKinematics` folder:
      This folder contains the outcomes of the second optimization and validation step.
      Specifically, the muscle kinematics states of the reference OpenSim model are saved as .pkl files.
      The muscle moment arm comparison plots between OpenSim and MuJoCo models are saved as .svg files.

   - `Step3_MuscleKinetics` folder:
      This folder contains the outcomes of the third optimization and validation step.
      Specifically, the muscle kinetic states of the reference OpenSim model are saved as .pkl files.
      The muscle force comparison plots between the reference OpenSim and MuJoCo models are saved as .svg files.

   - `[osim model name]_conversion.log` file:
      This log file saves all the log information while converting the OpenSim model. This is useful to understand what steps/processes the pipeline performed. When sunbmitting issues to the git repo, it is highly recommended to include this log file.

   - `[osim model name]_cvt1.xml` file:
      This is the converted MuJoCo model after the first conversion step.

   - `[osim model name]_cvt2.xml` file:
      This is the converted MuJoCo model after the second optimization step.

   - `[osim model name]_cvt3.xml` file:
      This is the converted MuJoCo model after the third optimization step. We recommend to always use this model, to have good muscle kinematics and kinetics.

   - `[osim model name]_conversion_report.pdf` file:
      This pdf file summarizes the validation of the converted MuJoCo model. Similarly, the three validation steps (vlt1, vlt2, vlt3) are presented. 
