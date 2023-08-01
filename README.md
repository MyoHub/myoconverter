<p align="center">
<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/logo-color-fit_horizontal.png?raw=true" width=800>
</p>

[![Documentation Status](https://readthedocs.org/projects/myoconverter/badge/?version=latest)](https://myoconverter.readthedocs.io/en/latest/)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](https://github.com/MyoHub/myoConverter/blob/main/docs/CONTRIBUTING.md)
[![License Badge](https://img.shields.io/hexpm/l/apa)](https://github.com/opensim-org/opensim-core/blob/master/LICENSE.txt)
<!-- [![Downloads](https://pepy.tech/badge/myosuite)](https://pepy.tech/project/myosuite)
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/1U6vo6Q_rPhDaq6oUMV7EAZRm6s0fD1wn?usp=sharing) -->

# myoConverter
**myoConverter** is a tool for converting OpenSim musculoskeletal (MSK) models to the MuJoCo model format with optimized muscle kinematics and kinetics. 

Building upon the foundation of the previous [O2MConverter](https://github.com/aikkala/O2MConverter) project, we extensively rewrote the functions, incorporated new features, and ensured compatibility with the latest OpenSim 4.0+ models. Additionally, two optimization steps were introduced to enhance the accuracy of muscle properties in both kinematics and kinetics.

We evaluate the accuracy of the converter with a handful of models. However, these models do not cover all possible features of OpenSim models. Hence, when converting a new model, there is a chance the conversion fails due to a missing implementation. In this case, you can open an issue, or, preferably, [contribute](https://github.com/MyoHub/myoConverter/blob/main/docs/source/participate.rst) to the project and create a pull request.


 [Example Model List]() | [External Model List](https://github.com/facebookresearch/myosuite/blob/main/docs/source/suite.rst#tasks) | [Documentation](https://myosuite.readthedocs.io/en/latest/)
| [myoSuite](https://sites.google.com/view/myosuite/myosim?authuser=0) | [Current Limitations](---)

## Example models
Here we present a collection of models, as examples, that have been processed with the myoConverter tool. We try to keep these converted models up-to-date (in case of bug fixes etc.), but it is recommended to run the conversions yourself to ensure up-to-date models.

|   | Model name| Source | Validation | Conversion Speed | 
|--------------------|--------------|:-------:|:--------:|-------|
|<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/tug-of-war.png" width="200">| [Turg of War](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/TugOfWar/Tug_of_War_cvt3.xml) <br> - 1 DoF <br> - 2 Muscles | [Osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim/TugOfWar)  |   [Report](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/TugOfWar/Tug_of_War_conversion_report.pdf) | Regular: 30 sec <br> Speedy: 30 sec |
|<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/arm26.png" width="200">| [Simple Arm](https://github.com/MyoHub/myo_sim/blob/main/elbow/myoelbow_2dof6muscles.xml)  <br> - 2 DoFs <br> - 6 Muscles | [Osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim/Arm26)  |   [Report](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Arm26/arm26_conversion_report.pdf) | Regular: 5 min 30 sec <br> Speedy: 4 min 37 sec |
|<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/leg6dof.png" width="200">| [Single Leg](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Leg6Dof9Musc/leg6dof9musc_cvt3.xml)   <br> - 6 DoFs <br> - 9 Muscles | [Osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim/Leg6Dof9Musc)  |   [Report](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Leg6Dof9Musc/leg6dof9musc_conversion_report.pdf) | Regular: 4 min 3 sec <br> Speedy: 3 min 41 sec |
|<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/gait10.png" width="200">| [2D Gait Model](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Gait10dof18musc/gait10dof18musc_cvt3.xml) <br> - 10 DoFs <br> - 18 Muscles | [Osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim/Gait10dof18musc)  |   [Report](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Gait10dof18musc/gait10dof18musc_conversion_report.pdf) | Regular: 8 min 21 sec <br> Speedy: 7 min 33 sec |
|<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/gait23.png" width="200">| [3D Gait Model](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Gait2354Simbody/gait2354_simbody_cvt3.xml)  <br> - 23 DoFs <br> - 54 Muscles | [Osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim/Gait2354Simbody)  |   [Report](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Gait2354Simbody/gait2354_simbody_conversion_report.pdf) | Regular: 34 min 21 sec <br> Speedy: 22 min 58 sec |
|<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/neck6d.png" width="200">| [Neck Model](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Neck6D/HYOID_1.2_ScaledStrenght_UpdatedInertia_adjusted_cvt3.xml) <br> - 6 DoFs <br> - 72 Muscles | [Osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim/Neck6D)  |   [Report](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/Neck6D/HYOID_1.2_ScaledStrenght_UpdatedInertia_adjusted_conversion_report.pdf) | Regular: 200 min 14 sec <br> Speedy: 57 min 43 sec |
<!--|<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/fullbody3d.png" width="200">| [Full Body Model](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/FullBody3D/FullBodyModel_Hamner2010_v2_0_cvt3.xml)  <br> - 36 DoFs <br> - 86 Muscles | [Osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim/FullBody3D)  |   [Report](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/FullBody3D/FullBodyModel_Hamner2010_v2_0_conversion_report.pdf) | Regular: 75 min 17 sec <br> Speedy: 58 min 46 sec | -->
<!--|<img src="https://github.com/MyoHub/myoConverter/blob/main/docs/source/images/wristhand.png" width="200">| [Wrist-Hand Model](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/WristHandModel/wrist_cvt3.xml)  <br> - 10 DoFs <br> - 25 Muscles | [Osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim/WristHandModel)  |   [Report](https://github.com/MyoHub/myoConverter/blob/main/models/mjc/WristHandModel/wrist_conversion_report.pdf) | Regular: 54 min 06 sec <br> Speedy: 18 min 26 sec | -->



MyoConverter has been extensively used to build up the [MyoSim](https://github.com/MyoHub/myo_sim) MSK models that have been used in the [MyoSuite](https://sites.google.com/view/myosuite) framework. These converted models underwent additional manual adjustments to address minor issues, ensuring their suitability for functional task simulations. 

Besides, a list of third-party converted models using this tool can be found [here]().

We encourage you to review these models before embarking on any redundant efforts. However, we must emphasize that we cannot guarantee the accuracy of these models, as there is no universal test for this. If you have specific concerns or questions regarding any of the models, we recommend reaching out directly to the respective model creators for further information and clarification.


## Download & Setup

### Linux
If you would like to convert your own MSK model, you can use myoConverter by following the steps outlined below:

- Clone the repo
```bash
git clone git@github.com:MyoHub/myoConverter.git; cd myoConverter
```


- Create a conda environment with the `conda_env.yml` file
```bash
conda env create -f conda_env.yml
conda activate myoConverter
```

- Test installation
```bash
python -c "import myoConverter"
```

### Windows/MacOS
In Windows and MacOS, we provide a docker image that has the myoConverter ready to be used. Please follow the following usage instructions:
.... 

## Quick example

#### Call a Python function

```python
from myoconverter import O2MPipeline
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)
```
converts "/path/to/*.osim" to a MuJoCo XML file and outputs the model into folder "/path/to/output/folder". The `**kwargs` may contain the optional parameters as listed below:
- `convert_steps` : Selected conversion steps, could be any subset of `[1, 2, 3]` based on the needs, Default = `[1, 2, 3]`
- `muscle_list` : Selected specific muscles for the conversion and validation steps, Default = None
- `osim_data_overwrite` : If true, overwrite extracted Osim model state files, Default = False [overwrite is needed, if Osim model has changed]
- `conversion` : If true, perform the conversion functions of selected steps, Default = True
- `validation` : If true, perform the validation functions of selected steps, Default = True
- `speedy` : If true, reduce the number of checking notes in optimization steps to increase speed, Default = False
- `generate_pdf` : If true, generate a pdf report of the validation results, Default = False
- `add_ground_geom` : If true, a geom (of type plane) is added to the MuJoCo model as ground, Default = False
- `treat_as_normal_path_point` : If true, MovingPathPoints and ConditionalPathPoints will be treated as normal PathPoints, Default = False

The following example can be used to setup the `**kwargs`

```python
# define pipeline configurations
kwargs = {}  # define kwargs inputs
kwargs['convert_steps'] = [1, 2, 3]             # All three steps selected
kwargs['muscle_list'] = None                    # No specific muscle selected, optimize all of them
kwargs['osim_data_overwrite'] = True            # Overwrite the Osim model state files
kwargs['conversion'] = True                     # Yes, perform 'Cvt#' process
kwargs['validation'] = True                     # Yes, perform 'Vlt#' process
kwargs['speedy'] = False                        # Do not reduce the checking notes to increase speed
kwargs['generate_pdf'] = True                   # Do not generate validation pdf report
kwargs['add_ground_geom'] = True                # Add ground to the model
kwargs['treat_as_normal_path_point'] = False    # Using constraints to represent moving and conditional path points
```

<!--
### Call from command line

To print usage information, input in a command line:

```bash
python myoconverter/O2MPipeline.py
```

An example of calling the converter script:

```bash
python myoconverter/O2MPipeline.py /path/to/osim.xml /path/to/geometry/folder /path/to/output/folder --speedy True --add_ground_geom True
```
The command line basically works the same way as when using the`O2MPipeline` function. 
-->

## Contribution
We highly encourage both users and experts to actively contribute to this open-source software. By sharing your insights and expertise, you can help enhance the functionality and maintenance of myoConverter for the benefit of all users. For more detailed information about the tool, please refer to the [documentation](https://github.com/MyoHub/myoConverter/blob/main/docs/source/index.rst).


## Citation
```BibTeX
@inproceedings{wang2022myosim,
  title={MyoSim: Fast and physiologically realistic MuJoCo models for musculoskeletal and exoskeletal studies},
  author={Wang, Huawei and Caggiano, Vittorio and Durandau, Guillaume and Sartori, Massimo and Kumar, Vikash},
  booktitle={2022 International Conference on Robotics and Automation (ICRA)},
  pages={8104--8111},
  year={2022},
  organization={IEEE}
}
```

```BibTeX
@inproceedings{ikkala2022converting,
  title={Converting biomechanical models from opensim to Mujoco},
  author={Ikkala, Aleksi and H{\"a}m{\"a}l{\"a}inen, Perttu},
  booktitle={Converging Clinical and Engineering Research on Neurorehabilitation IV: Proceedings of the 5th International Conference on Neurorehabilitation (ICNR2020), October 13--16, 2020},
  pages={277--281},
  year={2022},
  organization={Springer}
}
```

## License

The licenses and credits for the original OpenSim models can be found in each respective model folder in [myoConverter/models/osim](https://github.com/MyoHub/myoConverter/tree/main/models/osim). The code presented in this repository and the converted models are licensed with Apache 2.0.
