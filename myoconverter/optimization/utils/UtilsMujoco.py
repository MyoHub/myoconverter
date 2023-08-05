
import mujoco
import numpy as np
from numpy import pi
import copy
import itertools
from myoconverter.optimization.utils.UtilsRotation import spherical2cartesian,\
     quaternionRotaion, cylindarical2cartesian

# get all independed joint coordinate range
def getCoordinateRange_mjc(mjc_model):
    
    # extract joint names and ranges
    joint_names = [mujoco.mj_id2name(mjc_model, mujoco.mjtObj.mjOBJ_JOINT, idx) for idx in range(mjc_model.njnt)]
    joint_ranges = mjc_model.jnt_range
    
    free_jnt_id = []
    ang_ranges = {}
    
    for ira, ra in enumerate(joint_ranges):
        if mjc_model.eq_obj1id is None:  # if there are no equality joint constraints
            free_jnt_id.append(ira)
            ang_ranges[joint_names[ira]] = [round(ra[0], 4), round(ra[1], 4)]
        else:
            if not ra[0] == ra[1] and not ira in mjc_model.eq_obj1id[mjc_model.eq_type == 2]:
                free_jnt_id.append(ira)
                ang_ranges[joint_names[ira]] = [round(ra[0], 4), round(ra[1], 4)]
            
    return ang_ranges, free_jnt_id
    

def calculateEndPoints_mjc(mjc_model_path, end_points, n_eval):
    """
    Calculate the positions of endpoints when iterating all joint angle meshes.
    This is to check the Geometry and Joint Definition matches between the Opensim
    and the converted MuJoCo models.

    Parameters
    ----------
    osimModel_path : string
        Path and name of the Osim model that will be evaluated.
    endPoints : list of string
        The names of the the body whose positions will be evaluated.

    Returns
    -------
    evaData: dict
        The evaluation data, including the joint angle meshes and the corresponding
        endpoint locations
    """
    
    mjc_model  = mujoco.MjModel.from_xml_path(mjc_model_path) # load mj model

        # set up simulations
    mjc_data = mujoco.MjData(mjc_model)
    mjc_model.opt.timestep = 0.001 ## same time step as opensim
    
    # extract independed joint names and ranges
    ang_ranges, free_jnt_id = getCoordinateRange_mjc(mjc_model)
            
    # find the index of the endPoint for checking the position
    end_id = []
    for end_point in end_points:
        # Get idx of end_point; returns -1 if it doesn't exist in the model
        idx = mujoco.mj_name2id(mjc_model, mujoco.mjtObj.mjOBJ_SITE, end_point)
        if idx != -1:
            end_id.append(idx)
    
    # get the endBody positions by assign different joint angles
    end_pos = []
    for n in range(n_eval):
        
        jointEval = []
        
        for dof_id in free_jnt_id:
            
            # ang_range = ang_ranges[mujoco.mj_id2name(mjc_model, mujoco.mjtObj.mjOBJ_JOINT, dof)]
            ang_range = ang_ranges[mujoco.mj_id2name(mjc_model, mujoco.mjtObj.mjOBJ_JOINT, dof_id)]
            np.random.seed(n)

            jointEval.append((np.array(ang_range[0]) + 0.8*np.random.random(1)* \
                    (np.array(ang_range[1]) - np.array(ang_range[0])))[0])
            
        mjc_data.qpos[free_jnt_id] = jointEval
            
        dependencyJnts, dependencyJntAngs, freeJnts = dependencyJointAng(mjc_model, free_jnt_id, jointEval)
        mjc_data.qpos[dependencyJnts] = dependencyJntAngs

        # assign the value of locked joints
        lockedJnts, lockedJointAngs = lockedJointAng(mjc_model)
        mjc_data.qpos[lockedJnts] = lockedJointAngs

        # initialize vel and ctrl to 0
        mjc_data.qvel[:] = np.zeros(len(mjc_data.qvel),)
        mjc_data.ctrl[:] = np.zeros(len(mjc_data.ctrl),)

        # simulate and extract end point position
        mujoco.mj_step(mjc_model, mjc_data)

        end_pos.append(mjc_data.site_xpos[end_id].copy())
    
    return ang_ranges, end_pos

def dependencyJointAng(mjc_model, free_jnt_id_array, jnt_ang_array):
    
    couplingJnt = mjc_model.eq_type == 2
    activeEqu = mjc_model.eq_active == 1
    
    dependencyJntAngs = []
    dependencyJnts = []
    freeJnts = []
    
    # check if couplingJnt exist
    couplingJnt_sign = False

    if type(couplingJnt) == bool or type(couplingJnt) == np.bool_:
        if couplingJnt:
            couplingJnt_sign = True
    else:
        couplingJnt_sign = True

    # if exist, then extract them
    if couplingJnt_sign:
        for ind in range(len(couplingJnt)):
            if all([couplingJnt[ind], activeEqu[ind]]):
                
                dependencyJnt = mjc_model.eq_obj1id[ind]
                if dependencyJnt in dependencyJnts:
                    raise RuntimeError("multiple equality constraints for the same dependency joint..\n")
                
                freeJnt = mjc_model.eq_obj2id[ind]
                
                if freeJnt in free_jnt_id_array:
                    dependencyJnts.append(dependencyJnt)
                    freeJnts.append(freeJnt)
                else:
                    continue
                    
                freeJntAng = jnt_ang_array[np.where(free_jnt_id_array == freeJnt)[0][0]]
                
                poly_gains = mjc_model.eq_data[ind]
                
                dependencyJntAng = 0
                for idp, poly_gain in enumerate(poly_gains):
                    dependencyJntAng = dependencyJntAng + poly_gain*freeJntAng**idp
                    
                dependencyJntAngs.append(dependencyJntAng)
            
    return dependencyJnts, dependencyJntAngs, freeJnts

def lockedJointAng(mjc_model):
    """
    find the self-locked joints and corresponding joint angles.
    """

    # find locked constraints that satisfy all three conditions
    lockedCons = np.logical_and(np.logical_and(mjc_model.eq_type == 2, mjc_model.eq_active == 1), mjc_model.eq_obj2id == -1)

    lockedJnts = []
    lockedJntAngs = []

    # dirty fix when there are only one constraints!!
    lockedCons_sign = False

    if type(lockedCons) == bool or type(lockedCons) == np.bool_:
        if lockedCons:
            lockedCons_sign = True
    else:
        lockedCons_sign = True

    # run through all the locked constraints
    if lockedCons_sign:
        for ind in range(len(lockedCons)):
            if lockedCons[ind]:
                lockedJnts.append(mjc_model.eq_obj1id[ind])
                lockedJntAngs.append(mjc_model.eq_data[ind][0])

    return lockedJnts, lockedJntAngs


def sortMuscleWrapSiteJoint(mjc_model, muscle_para_osim):
    """
    find out the wrapping objects, siteside, and corresponding joints
    that should be optimized for a given muscle 
    
    Parameters
    ----------
    mjc_model: mujoco Model
        A mujoco model.
    muscle_para_osim: dict
        The muscle, wrapping_coordinates, coordinate_range information
        extracted from osim model.

    Returns
    -------
    muscle_para_osim: dict data
        add wrapping object, site side, information
    """

    WrapTypes=['NONE','JOINT','PULLEY','SITE','SPHERE','CYLINDER']  # default wrapping types

    muscle_name = muscle_para_osim['muscle_name']

    # remove the wrapping information for updating purpose
    muscle_para_osim['wrapping_info'] = len(muscle_para_osim['wrapping_coordinates'])*[None]

    # instead of looking for the index from muscle_list, the tendon id needs to 
    # be found from the tendon name list, due to the exists of liagments, so far
    # liagments are not optimized (may cause issue, TODO in the future)

    # Get tendon that corresponds to muscle_name (note: we can now have tendons that do not have actuators: liagments)
    it = mjc_model.actuator(muscle_name).trnid[0]

    # Not sure if I'm using trnid correctly above, so let's check if the correct tendon has been found. The name of the
    # tendon should be {muscle_name}_tendon.
    tendon_name = mujoco.mj_id2name(mjc_model, mujoco.mjtObj.mjOBJ_TENDON, it)
    if tendon_name != f"{muscle_name}_tendon":
        raise RuntimeError(f"Could not find correct tendon. Found tendon is called {tendon_name}, "
                           f"but the muscle is called {muscle_name}")
 
    tendon_adr_st = mjc_model.tendon_adr[it]  # get the index number of the tendon
    site_num = mjc_model.tendon_num[it]  # get the number of sites inside a tendon

    
    wrap_dict = {}
    site_list = []
    wrap_id_list = []

    for site_id in range(site_num):  # run through these sites

        i = tendon_adr_st + site_id
        
        # if the site is a wrap
        if mjc_model.wrap_type[i] == 4 or mjc_model.wrap_type[i] == 5:  

            wrap_id = mjc_model.wrap_objid[i]  # get the wrap id
            # get geometry name
            geom_name = mujoco.mj_id2name(mjc_model, mujoco.mjtObj.mjOBJ_GEOM, wrap_id)

            if not wrap_id in wrap_id_list:  # 
                wrap_id_list.append(wrap_id)
            
                wrapInfo = [geom_name,
                            wrap_id,
                            copy.deepcopy(mjc_model.geom_pos[wrap_id]),
                            copy.deepcopy(mjc_model.geom_size[wrap_id]),
                            copy.deepcopy(mjc_model.geom_quat[wrap_id]),
                            WrapTypes[mjc_model.wrap_type[i]],
                            []]

                if mjc_model.wrap_prm[i] != 0:  # if so, and there is side site
                    site_side_id = int(mjc_model.wrap_prm[i])                
                    siteInfo = [site_side_id, copy.deepcopy(mjc_model.site_pos[site_side_id])]

                else:
                    raise RuntimeError(f"There is no side site for the Wrap: {geom_name}, at muscle: {muscle_name}")

            # Below check which joints this wrapping object and wrapping point affectting...
            # By moving the wrapping object to extrem locations. Since the wrapping point
            # was already defined to the center of the wrapping object, muscle path should
            # always go through the wrapping object if affecting, which create moment arm
            # changes.
            # If no joints are affected by the wrapping object and point, then the location
            # of the wrapping point will be defined as [0, 0, 0]. This could happen, since
            # multiple wrapping points can be detected for the same muscle and same wrapping
            # object in the first step. 

            for i_jnts, joints in enumerate(muscle_para_osim['wrapping_coordinates']):

                # compute moment arms before change
                ma_org = computeMomentArmMuscleJoints(mjc_model, muscle_name, joints,\
                                                    muscle_para_osim['mjc_coordinate_ranges'][i_jnts], 3)

                # change the wrapping object location to a extrem location, 10 times of its size
                mjc_model.geom(wrap_id).pos = 10*mjc_model.geom(wrap_id).size[0] + mjc_model.geom(wrap_id).pos
                mjc_model.site(siteInfo[0]).pos = copy.deepcopy(mjc_model.geom(wrap_id).pos)

                # compute the moment arms gain after the change
                ma_aft = computeMomentArmMuscleJoints(mjc_model, muscle_name, joints,\
                                                    muscle_para_osim['mjc_coordinate_ranges'][i_jnts], 3)

                # Check if there are large changes before and after the wrap object location change
                # If there is moment arm differences, then save the wrapping object.
                ma_org_nonzeros = np.maximum(abs(ma_org.flatten()), 1e-3)
                if sum(abs((ma_aft.flatten() - ma_org.flatten())/ma_org_nonzeros)) > 1e-3:
                    
                    # And also check if the wrapping point is effecting the moment arms or not,
                    # if yes, then this wrapping point will be saved in the muscle_parameter
                    # for the moment arm optimization.

                    # move site position to its orignal location
                    mjc_model.site(siteInfo[0]).pos = siteInfo[1]

                    # calculate the moment arm again 
                    ma_pnt = computeMomentArmMuscleJoints(mjc_model, muscle_name, joints,\
                                            muscle_para_osim['mjc_coordinate_ranges'][i_jnts], 3)

                    # calculate the difference
                    ma_aft_nonzeros = np.maximum(abs(ma_aft.flatten()), 1e-3)
                    if sum(abs((ma_pnt.flatten() - ma_aft.flatten())/ma_aft_nonzeros)) > 1e-3:
                        wrapInfo[-1].append(siteInfo[0])
                        wrapInfo[-1].append(siteInfo[1])

                        # only save the wrapping objects and sites when both of them has effects
                        # on the joint moment arms. Here we ASSUME, at most only one wrapping object
                        # and one corresponding wrapping point for each muscle affecting on the checking
                        # joints
                        muscle_para_osim['wrapping_info'][i_jnts] = (wrapInfo)

                    else:
                        print('Wrapping sidesite does not affect moment arm, not saving...\n')

                # now move the wrapping object back to the origional location
                mjc_model.geom(wrap_id).pos = wrapInfo[2]

    return muscle_para_osim


def updateSiteSide(mjc_model):
    # change all the site side of the mujoco Model to the center of its wrap object

    # Get geom names and site names
    geom_names = [mujoco.mj_id2name(mjc_model, mujoco.mjtObj.mjOBJ_GEOM, idx) for idx in range(mjc_model.ngeom)]
    site_names = [mujoco.mj_id2name(mjc_model, mujoco.mjtObj.mjOBJ_SITE, idx) for idx in range(mjc_model.nsite)]
    
    # run through all geom items
    for geom in geom_names:
        if '_wrap' in geom:  # if '_wrap' is in the geom name, then it is a wrap object
            geom_location = mjc_model.geom(geom).pos
        
            # find corresponding site sides, and update site side locations
            for site in site_names:
                if geom[1:-5] + '_site' in site:
                    mjc_model.site(site).pos = geom_location
                    
    return mjc_model
                    

# def mjc_sim(mujocoModel):
#     # load mujoco simulation models
#     return mujoco_py.MjSim(mujocoModel)


def computeMomentArmMuscleJoints(mjc_model, muscle, joints, ang_ranges, evalN):
    '''
    Calculate moment arm matries from given muscles and joints
    '''
    
    if type(joints) != list:
        joints = [joints]
    
    mjc_data = mujoco.MjData(mjc_model)
    mjc_model.opt.timestep = 0.001 ## same time step as opensim
    
    muscles_idx = mujoco.mj_name2id(mjc_model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)  # muscle index
        
    joints_idx = []
    for joint in joints:
        joints_idx.append(mujoco.mj_name2id(mjc_model, mujoco.mjtObj.mjOBJ_JOINT, joint))
        
    ang_meshes=[]
    
    for ij, joint in enumerate(joints):
        ang_meshes.append(np.linspace(ang_ranges[ij][0], ang_ranges[ij][1], evalN))

    mom_arm =[]
    #  for im, muscle in enumerate(muscles):  does not run through muscles, only one
    for setAngleDofs in itertools.product(*ang_meshes):
        mjc_data.qpos[:] = np.zeros(len(mjc_data.qpos),)
        mjc_data.qvel[:] = np.zeros(len(mjc_data.qvel),)
        mjc_data.qpos[joints_idx] = setAngleDofs
        
        # assign the value of dependency joints
        dependencyJnts, dependencyJntAngs, freeJnts =\
            dependencyJointAng(mjc_model, joints_idx, setAngleDofs)
        mjc_data.qpos[dependencyJnts] = dependencyJntAngs

        # assign the value of locked joints
        lockedJnts, lockedJointAngs = lockedJointAng(mjc_model)
        mjc_data.qpos[lockedJnts] = lockedJointAngs

        mujoco.mj_step(mjc_model, mjc_data)
            
        mom_arm_sub = mjc_data.actuator_moment[muscles_idx, joints_idx].copy()
        
        # if dependencyJnts: 
        #     # if there are dependency joint, then calculate moment arm using
        #     # finite difference method, based on dTendon_Len/dJoint_ang
            
        #     # first get the muscle length with current joint angles
        #     mus_length0 = mjc_data.actuator_length[muscles_idx].copy()
            
        #     setAngleDofs_copy = list(setAngleDofs)
            
        #     # then a small change in every dependented free joint
        #     for fjoint in freeJnts:
        #         # first set everything to 0
        #         mjc_data.qpos[:] = np.zeros(len(mjc_data.qpos),)
        #         mjc_data.qvel[:] = np.zeros(len(mjc_data.qvel),)
                
        #         # find the index of this free joint
        #         ifj = joints_idx.index(fjoint)
                
        #         if setAngleDofs[ifj] == ang_ranges[ifj][1]:  # if equal to the upper bounds
        #             setAngleDofs_copy[ifj] = setAngleDofs[ifj] - (ang_ranges[ifj][1] - ang_ranges[ifj][0])*0.005  # then change down
        #         else:
        #             setAngleDofs_copy[ifj] = setAngleDofs[ifj] + (ang_ranges[ifj][1] - ang_ranges[ifj][0])*0.005  # else change up
                    
        #         # assign to the sim data
        #         mjc_data.qpos[joints_idx] = setAngleDofs_copy
    
        #         dependencyJnts, dependencyJntAngs, freeJnts =\
        #                 dependencyJointAng(mjc_model, joints_idx, setAngleDofs_copy)
        #         mjc_data.qpos[dependencyJnts] = dependencyJntAngs

        #          # assign the value of locked joints
        #         lockedJnts, lockedJointAngs = lockedJointAng(mjc_model)
        #         mjc_data.qpos[lockedJnts] = lockedJointAngs
        
        #         # one step forward simulation
        #         mujoco.mj_step(mjc_model, mjc_data)
                
        #         mus_length1 = mjc_data.actuator_length[muscles_idx].copy()
                
        #         if setAngleDofs[ifj] == ang_ranges[ifj][1]:  # if equal to the upper bounds
        #             mom_arm_sub[ifj] = -(mus_length1 - mus_length0)/((ang_ranges[ifj][1] - ang_ranges[ifj][0])*0.005)
        #         else:
        #             mom_arm_sub[ifj] = (mus_length1 - mus_length0)/((ang_ranges[ifj][1] - ang_ranges[ifj][0])*0.005)
                    
        #         setAngleDofs_copy[ifj] = setAngleDofs[ifj]  # change back to the initial length
                
        mom_arm.append(mom_arm_sub)
        
    return np.array(mom_arm)

def updateWrapSites(mjc_model, wrap_type, wrap_id, pos_wrap, size_wrap, rot_wrap, side_ids, vec_optimized):
    
    """
    Applies values contained in vec_optimized to optimizing side of mujoco model 
    
    Parameters:
        mjc_model (mujoco_py.cymj.PyMjModel): MuJoCo model
        wrap_type (numpy.list): 1D list of string, clarify the wrapping types
        side_ids (numpy.list): 1D list with the id of sides
        pos (numpy.list): a list of 3D arrays with the position of the wrapping objects
        size (numpy.list): a list of 3D arrays with the size of wrapping objects
        rotation (numpy.list): a list of 4D arrays with the rotation info of wrapping objects
        torus_flag (numpy.list): a list of strings, clarify whether is the origional wrapping torus
        vec_optimized (numpy.ndarray): 1D vector with the concatenated array of optimized par
    """
    # make sure side_ids is list
    if type(side_ids) != list:
        side_ids = [side_ids]
    
    if wrap_type == "TORUS":
        
        # assign the center location of the wrap as the location of the sites
        for side_id in side_ids:
            mjc_model.site(side_id).pos = pos_wrap
    
    
    elif wrap_type == 'ELLIPSOID_CYLINDER':
        
        # update wrap size, location, rotation, as well as the site locations
        # TODO: add rotation update, once optimized
        
        mjc_model.geom(wrap_id).size[0:2] = vec_optimized[0:2]
        mjc_model.geom(wrap_id).pos = vec_optimized[2:5]
        
        for isi, side_id in enumerate(side_ids):
            cylinderical_side = np.array([1.2*vec_optimized[0], vec_optimized[5 + 2*isi], vec_optimized[5 + 2*isi + 1]])
            cartesian_side = cylindarical2cartesian(np.array([0, 0, 0]), cylinderical_side)
            rotation_side = quaternionRotaion(rot_wrap, cartesian_side)  + pos_wrap
            mjc_model.site(side_id).pos = rotation_side
        
    elif wrap_type == 'ELLIPSOID_SPHERE':
        
        # update wrap size, location, as well as the site locations
        mjc_model.geom(wrap_id).size[0] = vec_optimized[0]
        mjc_model.geom(wrap_id).pos = vec_optimized[1:4]
        
        for isi, side_id in enumerate(side_ids):
            spherical_side = np.array([1.2*vec_optimized[0], vec_optimized[4 + 2*isi], vec_optimized[4 + 2*isi + 1]])
            cartesian_side = spherical2cartesian(np.array([0, 0, 0]), spherical_side)
            rotation_side = quaternionRotaion(rot_wrap, cartesian_side) + pos_wrap
            mjc_model.site(side_id).pos = rotation_side
    
    elif wrap_type == 'SPHERE':
        
        # update only the site locations
        for isi, side_id in enumerate(side_ids):
            spherical_side = np.array([1.2*size_wrap[0], vec_optimized[0 + 2*isi], vec_optimized[0 + 2*isi + 1]])
            cartesian_side = spherical2cartesian(np.array([0, 0, 0]), spherical_side)
            rotation_side = quaternionRotaion(rot_wrap, cartesian_side) + pos_wrap
            mjc_model.site(side_id).pos = rotation_side
            
    elif wrap_type == 'CYLINDER':
        
        # update only the site locations
        for isi, side_id in enumerate(side_ids):
            cylinderical_side = np.array([1.2*size_wrap[0], vec_optimized[0 + 2*isi], vec_optimized[0 + 2*isi + 1]])
            cartesian_side = cylindarical2cartesian(np.array([0, 0, 0]), cylinderical_side)
            rotation_side = quaternionRotaion(rot_wrap, cartesian_side)  + pos_wrap
            mjc_model.site(side_id).pos = rotation_side
            
    return mjc_model

def getMuscleForceLengthCurvesSim(mjc_model, muscle, joints, jnt_arr, act_arr):
    """
    Simplification Version
    
    Given as INPUT an OpenSim muscle OSModel, a muscle variable and a nr
    of evaluation points this function returns as
    musOutput a vector of the muscle variable of interest in the converted MuJoCo model
    obtained by sampling the ROM of the joint spanned by the muscle in
    N_EvalPoints evaluation points.
    For multidof joint the combinations of ROMs are considered.
    For multiarticular muscles the combination of ROM are considered.
    The script is totally general because based on generating strings of code
    correspondent to the encountered code. The strings are evaluated at the end.
    """
    
    # %======= SETTINGS ======
    
    # find out the coupling DoFs of this muscle, so that the optimization of the
    # side site of corresponding DoFs only use the corresponding ma data. This 
    # will increase the speed of the optimzation, and avoid optimize something 
    # that is not needed, for instance, if only 2 out off 5 DoFs have ma difference
    # then the others are not necessary to be optimized.
    # osimDoF_cpJoints = getCouplingJoints(osimModel, muscles.get(curr_mus_name), joints)
    
    joints_idx = []
        
    if type(joints) != list:
        joints = [joints]
    
    if type(jnt_arr) != list:
        jnt_arr = [jnt_arr]
        
    if type(act_arr) != np.ndarray and type(act_arr) != list:
        act_arr = [act_arr]
        
    for joint in joints:
        joints_idx.append(mujoco.mj_name2id(mjc_model, mujoco.mjtObj.mjOBJ_JOINT, joint))
        
    force_mtu = np.zeros((len(act_arr), len(jnt_arr)))
    length_mtu = np.zeros((len(act_arr), len(jnt_arr)))
    
    muscle_id = mujoco.mj_name2id(mjc_model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle)

    mjc_model.actuator_dyntype[muscle_id] = 0 # no activation dynamics
    
    mjc_data = mujoco.MjData(mjc_model)
    mjc_model.opt.timestep = 0.001 ## same time step as opensim
        
    for ij, setAngleDofs in enumerate(jnt_arr):
        
        mjc_data.qpos[:] = np.zeros(len(mjc_data.qpos),)
        mjc_data.qvel[:] = np.zeros(len(mjc_data.qvel),)
        mjc_data.qpos[joints_idx] = setAngleDofs
        
        dependencyJnts, dependencyJntAngs, freeJnts =\
            dependencyJointAng(mjc_model, joints_idx, setAngleDofs)
        mjc_data.qpos[dependencyJnts] = dependencyJntAngs

        # find the self-locked joints and assign the contraint values
        lockedJnts, lockedJntAngs = lockedJointAng(mjc_model)
        mjc_data.qpos[lockedJnts] = lockedJntAngs
        
        for ia, act in enumerate(act_arr):
            mjc_data.ctrl[muscle_id] = act    # set control signal to 1
            # sim.data.act[actuator_id] = act  # directly assign activation to 1
        
            mujoco.mj_step(mjc_model, mjc_data)
            
            force_mtu[ia, ij] = mjc_data.actuator_force[muscle_id].copy()
            length_mtu[ia, ij] = mjc_data.actuator_length[muscle_id].copy()
    
    mjc_model.actuator_dyntype[muscle_id] = 3 # change back to activation dynamics 3
        
    return force_mtu, length_mtu
    
def updateMuscleForceProperties(mjc_model, muscle, res):
    """
    Update mujoco muscle property parameters with optimization results.
    """
    
    mjc_model.actuator(muscle).biasprm[2] = res[3]
    mjc_model.actuator(muscle).biasprm[0] = res[0]
    mjc_model.actuator(muscle).biasprm[1] = res[1]
    mjc_model.actuator(muscle).biasprm[7] = res[2]
    
    mjc_model.actuator(muscle).gainprm[2] = res[3]
    mjc_model.actuator(muscle).gainprm[0] = res[0]
    mjc_model.actuator(muscle).gainprm[1] = res[1]
    mjc_model.actuator(muscle).gainprm[7] = res[2]
    
    return mjc_model

