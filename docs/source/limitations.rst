.. _Limitations:

Limitations
===========

There are some limitations involved in the current conversion and optimisation process, that are good to keep in mind when using the converted and optimised models. Below we list these limitations, in no particular order.

1. **The converter assumes that the OpenSim XML model definition only has one kinematic tree.**

2. **Contact definitions are not converted.**

   - With the exception that if a model is converted with the `add_ground_geom` keyword, contacts between the model and ground are included. However, it is fairly easy to add contacts to the converted MuJoCo model afterwards, e.g., using MuJoCo's *contact/pair* definitions. One could also enable all contacts by setting `collision="all"` in the converted XML model file (by default we set `collision="predefined"`), but this could lead to unwanted collisions as most of the OpenSim models haven't been developed for this purpose and their collision geometries may not be suitable.

3. **Default values for parameters set in the *defaults* element of an OpenSim model are not parsed or used during the conversion.**

4. **Current conversion of conditional and moving path points may result in inaccurate movement of tendons.**
   - A detailed explanation of this conversion process can be found in :ref:`Model Conversion`. Addressing these inaccuracies are left for future work.

5. **There are multiple parsers that haven't been implemented, mainly related to actuators.**
   - The parsers that we know are missing are *BushingForce*, *FunctionBasedBushingForce*, *CoordinateLimitForce*, *PointActuator*, and *TorqueActuator*, but there likely are others that we haven't encountered yet.

6. **Non-linear dependencies between independent coordinates and dependent coordinates in a CustomJoint are not currently supported.**

7. **Wrapping object orientations are not optimized.**

   - As described in the section on :ref:`Muscle Kinematics Optimization`, we employ different optimization approaches based on the type of wrapping object involved. In the case of an ellipsoid wrapping object, since MuJoCo does not support this specific type, we convert it to either a cylinder or a sphere based on the axis ratio. During the optimization phase, we not only optimize how the muscle wraps over the wrapping objects but also adjust the position of the converted cylinder or sphere to obtain a more accurate approximation of the wrapping surface shape.

   - It's important to note that the orientation of the wrapping objects theoretically impacts the shape of the surface over which the muscle wraps. However, optimizing the rotation parameters introduces significant non-linearities that make the optimization process more challenging. Therefore, the current pipeline does not optimize the orientation of the wrapping objects. Consequently, the muscle moment arms for the converted cylinder or sphere may not be identical to those of the original ellipsoid wrapping objects.
  

8. **If the reference Opensim model has moment arm jumps, optimization does not smooth out the jumps before fitting to it.**

   - During our conversion process, we have observed sudden moment arm jumps (sign changes) in the reference OpenSim models. In the Muscle Kinematics Optimization step, we currently do not differentiate these jumps but accept them as part of the reference model.

   - There are a couple of reasons for this approach. Firstly, we have not yet found an effective method to accurately identify and isolate these moment arm jumps. Secondly, these jumps often occur when a muscle path point intersects with a wrapping object, causing the path to transition from one side of the wrapping object to the other.

   - It's important to note that simply smoothing the reference moment arm maps will not resolve this issue. This is because the same phenomenon occurs in the MuJoCo model, as the converted path points behave in a similar manner to the original model.

   - Addressing these moment arm jumps requires further investigation and development of techniques to properly handle the situation. As of now, we continue to work towards finding a solution to this challenge in order to improve the accuracy and fidelity of the converted models.

9. **If the muscle operates in a strange range, outside the range of [0 1.8], the optimization cannot find a good solution.**

   - In contrast to OpenSim, MuJoCo uses `operational range <https://mujoco.readthedocs.io/en/stable/modeling.html#muscle-actuators>`_ rather than optimal fiber length to define muscle force properties. The operational range represents the valid range of muscle activation values within which the muscle behaves in a reasonable manner.

   - In our optimization setup, we assume that the maximum operational range of the muscles falls within the interval [0, 1.8]. This range is considered reasonable as it ensures that the muscle behavior remains within acceptable biological constraints. It is important to note that this setup may not account for extreme conditions, such as the `Tug of War <https://github.com/MyoHub/myoconverter/tree/main/models/osim/TugOfWar>`_ model. However, it serves as a suitable constraint for typical biological musculoskeletal models.

10. **Current MuJoCo model uses non-elastic tendon.**

   - The introduction of elastic tendon muscle actuators in the recent MuJoCo update (as discussed in the `GitHub issue <https://github.com/deepmind/mujoco/issues/305>`_) is an exciting development. The inclusion of elastic muscle models in the conversion process would indeed contribute to creating more accurate and realistic models.

   - By incorporating elastic tendon muscle actuators, the converted models can better capture the behavior and properties of muscles with elastic tendons. This enhancement would allow for more accurate simulations and analyses, particularly in scenarios where muscle elasticity plays a significant role, such as during movements involving stretching or energy storage/release.
