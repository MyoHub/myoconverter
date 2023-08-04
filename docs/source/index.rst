Welcome to MyoConverters's documentation!
=========================================

**MyoConverter** is a software to convert and optimize biomechanical `OpenSim <https://simtk.org/projects/opensim/>`_ models to the `MuJoCo <http://www.mujoco.org/>`_ physics engine.

This software is intended for OpenSim 4.x models. If you have an OpenSim 3.x model, or earlier, it is recommended to update the model to the 4.x format using OpenSim's software. If this is not possible, then we suggest you try out the `O2MConverter <https://github.com/aikkala/O2MConverter>`_ which converts OpenSim 3.x models, but does not include moment arm and force optimizations. 


.. note::

   We evaluate the accuracy of the converter with a `handful of models <models.html>`_. However, these models do not cover all possible features of OpenSim models. Hence, when converting a new model, there is a chance the conversion fails due to a missing implementation. In this case you can open an issue, or, preferably, `contribute <participate.html>`_ to the project and create a pull request. `See the GitHub repository <https://github.com/MyoHub/myo-converter>`_

.. note::
  The example models may not be suitable for research purposes as such; you may need to further modify e.g. joint limits, or add contacts according to your use case. 

.. toctree::
   :maxdepth: 2
   :caption: Contents

   pipeline
   models
   install
   limitations
   plans
   participate
   
.. toctree::
   :maxdepth: 1
   :caption: References

   publications


How to cite
-----------

The conversion and optimization processes are largely built on the work presented in these two following papers. Please cite these papers if you use our software or the converted and optimized models provided by our software.

.. code-block:: text

  @inproceedings{wang2022myosim,
    title={MyoSim: Fast and physiologically realistic MuJoCo models for musculoskeletal and exoskeletal studies},
    author={Wang, Huawei and Caggiano, Vittorio and Durandau, Guillaume and Sartori, Massimo and Kumar, Vikash},
    booktitle={2022 International Conference on Robotics and Automation (ICRA)},
    pages={8104--8111},
    year={2022},
    organization={IEEE}}

.. code-block:: text

  @inproceedings{ikkala2022converting,
    title={Converting biomechanical models from OpenSim to MuJoCo},
    author={Ikkala, Aleksi and H{\"a}m{\"a}l{\"a}inen, Perttu},
    booktitle={Converging Clinical and Engineering Research on Neurorehabilitation IV: Proceedings of the 5th International Conference on Neurorehabilitation (ICNR2020)},
    pages={277--281},
    year={2022},
    organization={Springer}}

