.. _Muscle Kinematics Optimization:



Step 2: Muscle Kinematics Optimization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. default-role:: math
.. include:: <isonum.txt>

This step is to refine muscle paths, so that the muscle lengths are similar in both OpenSim and MuJoCo models when they are at the same body (joint) postures. 

A critical variable affecting muscle lengths is the movement arm. Moment arm reflects how muscle wraps over the joint of interest `[Michael A. Sherman] <https://asmedigitalcollection.asme.org/IDETC-CIE/proceedings-abstract/IDETC-CIE2013/55973/256002>`_. It can be repsent by equation: :math:`r{\theta } = dL/d{\theta }`

where :math:`L` represent the muscle length; :math:`\theta` is the joint angle; :math:`r{\theta}` is the moment arm at joint angle :math:`\theta`.

In the first conversion step, the muscle attaching points of converted MuJoCo models (either fixed or moving) are very well represented. 

However, the *wrapping objects* and the corresponding *site side* in converted MuJoCo models may not very well matched with the referencing OpenSim models, due to the fact that MuJoCo support less wrapping object shapes and has a different way of defining how muscle wraps over the wrapping objects. 

Therefore, in this second step, we focus on refining how muscle wraps over the wrapping object (by adjusting the *site side*). There are several scenarios: 

1. Cylinder wrapping object:
  - Cylinder wrapping objects have the same surface definition inside MuJoCo and OpenSim mdoels. Optimization only need to decide how the muscle wraps over it, which defined by the location of site side.

2. Sphere wrapping object:
  - Sphere wrapping objects have the same surface definition inside MuJoCo and OpenSim mdoels. Optimization only need to decide how the muscle wraps over it, which defined by the location of site side.

3. Ellipsoid wrapping object:
  - There is no ellipsoid wrapping object in MuJoCo, therefore, ellipsoid wrapping objects in OpenSim are converted to either cylinder or sphere, depends on the long-short axis ratio. Since the wrapping surfaces are not identifical anymore, in optimization both the wrapping object location and site side are optimized at the same time to achieve the best fit.
    
4. Torus wrapping object:
  - In MuJoCo, there is no torus wrapping object as well. A good approximation is to use a small sphere to simulate the hole of the torus. Then we define that the muscle path has to go through the sphere. No optimizaiton is needed in this case. 
