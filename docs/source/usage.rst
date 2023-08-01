Usage
============

.. _usage:

You don't need to install MyoConverter in order to use the optimized models. The conversion and optimization pipeline outputs the models as MuJoCo XML files, and these files have been uploaded to the GitHub repository, from where you can download them.


If you want to convert and optimize a model that has not been uploaded to the GitHub repository, or participate in the development of the pipeline, you only need to pull the project from GitHub, and set up the Conda environment. 

.. code-block:: bash

   git clone git@github.com:MyoHub/myoConverter.git
   conda env create -n myoconverter -f myoConverter/conda_env.yml
   conda activate myoconverter
   
Then you can run the pipeline by calling the python function.

.. code-block:: python

   from myoconverter.O2MPipeline import O2MPipeline
   
   # define pipline configurations
   kwargs = {}  # define kwargs inputs
   kwargs['convert_steps'] = [1, 2, 3]    # All three steps selected
   kwargs['muscle_list'] = None           # No specific muscle selected, optimize all of them
   kwargs['osim_data_overwrite'] = True   # Overwrite the Osim model state files
   kwargs['conversion'] = True               # Yes, perform 'Cvt#' process
   kwargs['validation'] = True            # Yes, perform 'Vlt#' process
   kwargs['speedy'] = False               # Do not reduce the checking notes to increase speed
   kwargs['generate_pdf'] = True         # Do not generate validation pdf report
   kwargs['add_ground_geom'] = True       # Add ground to the model
   kwargs['treat_as_normal_path_point'] = False    # Using constraints to represent moving and conditional
   
   O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)

Here are the explanations of these configuration files:

- *convert_steps* :  Selected conversion steps, could be any subset of [1, 2, 3] based on the needs, Default = [1, 2, 3]
- *muscle_list* :  Selected specific muscles for the conversion and validation steps', Default = None
- *osim_data_overwrite* :  If ture, overwrite extracted Osim model state files, Default = False [overwrite is needed, if Osim model has changed]
- *conversion* :  If true, perform the conversion functions of selected steps, Default = True
- *validation* :  If true, perform the validation functions of selected steps, Default = True
- *speedy* :  If true, reduce the number of checking notes in optimization steps to increase speed, Default = False
- *generate_pdf* :  If true, generate a pdf report of the validation results, Default = False
- *add_ground_geom* :  If true, a geom (of type plane) is added to the MuJoCo model as ground, Default = False
- *treat_as_normal_path_point* :  If true, MovingPathPoints and ConditionalPathPoints will be treated as normal PathPoints, Default = False

