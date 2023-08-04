.. _Muscle Kinetics Optimization:

Step 3: Muscle Kinetics Optimization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In this step, muscle parameters related to force-length relationships of MuJoCo models were adjusted to achieve similar muscle kinetic properties as OpenSim models. 

MuJoCo does not explictly use `optimal fiber length <https://simtk-confluence.stanford.edu:8443/display/OpenSim/Thelen+2003+Muscle+Model>`_, but the `operation range <https://mujoco.readthedocs.io/en/stable/modeling.html#muscle-actuators>`_ to define the force-length property. Even though both ways can well represent how muscle force changes with respect to length changes, the parameter mapping is not straight forward, especially considering that MuJoCo muscle model uses rigid-tendon. 

Therefore, in this optimization step, we optimize 4 muscle parameters: the muscle operation range :math:`[range0, range1]`, maximum active force :math:`Fmax`, and maximum passive force :math:`Fp,max`.

Boundaries of these four optimized parameters are:

.. math::

 0.1 < range0 < 1 \\
 1 < range1 < 1.9 \\
 0.5*Fmax0 < Fmax < 1.5*Fmax0 \\
 0.1*Fmax0 < Fp,max < 1.8*Fmax0

Several other parameters were preset for all muscles:
  - The active force-length curve maximum range :math: `[lmin, lmax]` are set as :math: `[0, 2]`.
  - The :math: `vmax` is set as 10*:math: `L0` (virtual optimal fiber length)
  - The :math: `Fv,max` is set as 1.4

Note: OpenSim and MuJoCo simulators share a similar way of define muscle force-velocity relationship, parameters can be directly mapped.
