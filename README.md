<p align="center">
<img src="docs/source/images/myconverter_logo.png?raw=true" width=800>
</p>

[![Documentation Status](https://readthedocs.org/projects/myoconverter/badge/?version=latest)](https://myoconverter.readthedocs.io/en/latest/)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](docs/source/participate.rst)
[![License Badge](https://img.shields.io/hexpm/l/apa)](https://github.com/opensim-org/opensim-core/blob/master/LICENSE.txt)
<!-- [![Downloads](https://pepy.tech/badge/myosuite)](https://pepy.tech/project/myosuite)
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/drive/1U6vo6Q_rPhDaq6oUMV7EAZRm6s0fD1wn?usp=sharing) -->

# MyoConverter
**MyoConverter** is a tool for converting OpenSim musculoskeletal (MSK) models to the MuJoCo model format with optimized muscle kinematics and kinetics.

Building upon the foundation of the previous [O2MConverter](https://github.com/aikkala/O2MConverter) project, we extensively rewrote the functions, incorporated new features, and ensured compatibility with the latest OpenSim 4.0+ models. Additionally, two optimization steps were introduced to enhance the accuracy of muscle properties in both kinematics and kinetics.

See the [documentation](https://myoconverter.readthedocs.io/en/latest/index.html) for more information about the converter and the conversion process.

 [Model List](https://myoconverter.readthedocs.io/en/latest/models.html) | [Documentation](https://myoconverter.readthedocs.io/en/latest/index.html)
| [MyoSuite](https://sites.google.com/view/myosuite/myosim?authuser=0) | [Current Limitations](https://myoconverter.readthedocs.io/en/latest/limitations.html)

## Example models
Here we present a few example models that have been processed with the MyoConverter tool. We try to keep these converted models up-to-date (in case of bug fixes etc.), but it is recommended to run the conversions yourself to ensure up-to-date models.

|   | Model name| Source | Validation | Conversion Speed* |
|--------------------|--------------|:-------:|:--------:|-------|
|<img src="docs/source/images/tug-of-war.png" width="200">| [Tug of War](models/mjc/TugOfWar/tugofwar_cvt3.xml) <br> - 1 DoF <br> - 2 Muscles | [Osim](https://github.com/MyoHub/myoconverter/tree/main/models/osim/TugOfWar)  |   [Report](models/mjc/TugOfWar/tugofwar.pdf) | Regular: 2 seconds |
|<img src="docs/source/images/arm26.png" width="200">| [Simple Arm](https://github.com/MyoHub/myo_sim/blob/main/elbow/myoelbow_2dof6muscles.xml)  <br> - 2 DoFs <br> - 6 Muscles <br> * *In MyoSim* | [Osim](https://github.com/MyoHub/myoconverter/tree/main/models/osim/Arm26)  |   [Report](models/mjc/Arm26/arm26.pdf) | Regular: 11 sec |
|<img src="docs/source/images/leg6dof.png" width="200">| [Single Leg](models/mjc/Leg6Dof9Musc/leg6dof9musc_cvt3.xml)   <br> - 6 DoFs <br> - 9 Muscles | [Osim](https://github.com/MyoHub/myoconverter/tree/main/models/osim/Leg6Dof9Musc)  |   [Report](models/mjc/Leg6Dof9Musc/leg6dof9musc.pdf) | Regular: 37 sec |
|<img src="docs/source/images/gait10.png" width="200">| [2D Gait Model](models/mjc/Gait10dof18musc/gait10dof18musc_cvt3.xml) <br> - 10 DoFs <br> - 18 Muscles | [Osim](https://github.com/MyoHub/myoconverter/tree/main/models/osim/Gait10dof18musc)  |   [Report](models/mjc/Gait10dof18musc/gait10dof18musc.pdf) | Regular: 60 sec |
|<img src="docs/source/images/gait23.png" width="200">| [3D Gait Model](models/mjc/Gait2354Simbody/gait2354_cvt3.xml)  <br> - 23 DoFs <br> - 54 Muscles | [Osim](https://github.com/MyoHub/myoconverter/tree/main/models/osim/Gait2354Simbody)  |   [Report](models/mjc/Gait2354Simbody/gait2354.pdf) | Regular: 5 min 35 sec |
|<img src="docs/source/images/neck6d.png" width="200">| [Neck Model](models/mjc/Neck6D/neck6d_cvt3.xml) <br> - 6 DoFs <br> - 72 Muscles | [Osim](https://github.com/MyoHub/myoconverter/tree/main/models/osim/Neck6D)  |   [Report](models/mjc/Neck6D/neck6d.pdf) | Regular: 54 min 02 sec |


Please also see [this list of models](https://myoconverter.readthedocs.io/en/latest/models.html) converted using MyoConverter.

**_NOTE:_** Speed test was done on a *12th Gen Intel® Core™ i5-12500H × 16*. Parallel computing is implemented, so more Threads in CPU will lead to faster conversion.    

**_NOTE:_** The converted XML model contains a keyframe which should be used when initialising the model. This keyframe sets the joint values such that all the joint/muscle path constraints are met. However, MuJoCo does not load the keyframe by default. When using the MuJoCo `simulate` GUI, please hit the `Load key` button to load the keyframe. When loading the model using MuJoCo P  ython bindings, you can use following functions to load the keyframe:
```python
import mujoco
model = mujoco.MjModel.from_xml_path("path/to/model.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
```

## Download & Setup

We recommend installing MyoConverter via conda / mamba if you're running Linux (tested on Ubuntu 20.04 & 22.04). In earlier development phases we encountered issues in Windows. Hence, for Windows / MacOS users, we provide a docker image (follow [this link](./docker/README.md) for more instructions), which contains the tested Linux setup. If you try the conda / mamba approach on Windows / MacOS, please let us know how it goes!

### conda / mamba

- Clone the repo
```bash
git clone git@github.com:MyoHub/myoconverter.git; cd myoconverter
```

- Create a conda environment with the `conda_env.yml` file
```bash
conda env create -n myoconverter -f conda_env.yml
conda activate myoconverter
```

**Note** conda is very slow in solving the dependencies and installing the environment (>15 minutes). We recommend installation via [mamba](https://mamba.readthedocs.io/en/latest/installation.html) instead, which installs the environment in a couple of minutes. With mamba, the environment is created by replacing `conda` with `mamba`:
```bash
mamba env create -n myoconverter -f conda_env.yml
mamba activate myoconverter
```

- Add MyoConverter project folder to PYTHONPATH
```bash
export PYTHONPATH=${PYTHONPATH}:/path/to/myoconverter
```

- Optional: Test installation by running a model unit test
  
```bash
python myoconverter/tests/model_unit_test.py
```


## Quick example

#### Call a Python function

```python
from myoconverter.O2MPipeline import O2MPipeline

# define pipeline configurations
kwargs = {}  # define kwargs inputs
kwargs['convert_steps'] = [1, 2, 3]    # All three steps selected
kwargs['muscle_list'] = None           # No specific muscle selected, optimize all of them
kwargs['osim_data_overwrite'] = True   # Overwrite the Osim model state files
kwargs['conversion'] = True            # Yes, perform 'Cvt#' process
kwargs['validation'] = True            # Yes, perform 'Vlt#' process
kwargs['speedy'] = False               # Do not reduce the checking notes to increase speed
kwargs['generate_pdf'] = True          # Do not generate validation pdf report
kwargs['add_ground_geom'] = True       # Add ground to the model
kwargs['treat_as_normal_path_point'] = False    # Using constraints to represent moving and conditional path points

############### Simple Arm 2 DoFs 6 Muscles ################ 
osim_file = './models/osim/Arm26/arm26.osim'
geometry_folder = './models/osim/Arm26/Geometry'
output_folder = './models/mjc/Arm26'
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)
```

More conversion examples can be found in the [example folder](https://github.com/MyoHub/myoconverter/tree/main/examples). Detailed description of the conversion setup/process is in the [documentation](https://myoconverter.readthedocs.io/en/latest/pipeline.html).


## Contribution
We highly encourage both users and experts to actively contribute to this open-source software. By sharing your insights and expertise, you can help enhance the functionality and maintenance of MyoConverter for the benefit of all users. For more detailed information about the tool, please refer to the [documentation](https://myoconverter.readthedocs.io/en/latest/index.html).


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

The licenses and credits for the original OpenSim models can be found in each respective model folder in [models/osim](https://github.com/MyoHub/myoconverter/tree/main/models/osim). The code presented in this repository and the converted models are licensed with Apache 2.0.
