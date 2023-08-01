import opensim
import numpy as np
import itertools
      
def getJointsControlledByMuscle(osimModel, OSMuscle):
    """
    remove the locked coordinates and dependency coordinates of the spanned joints
    """

    currentState = osimModel.initSystem()

    #getting the joint crossed by a muscle
    muscleCrossedJointSet, _ = getJointsSpannedByMuscle(osimModel, OSMuscle.getName())
    
    #index f+or effective dofs
    DOF_Index = []
    # CoordinateBoundaries = []
    # degIncremList = [] 


    for n_joint in range(len(muscleCrossedJointSet)):

        # current joint

        curr_joint = muscleCrossedJointSet[n_joint]


        # Initial estimation of the nr of Dof of the CoordinateSet for that
        # joint before checking for locked and constraint dofs.
        nDOF = osimModel.getJointSet().get(curr_joint).numCoordinates()

        # skip welded joint and removes welded joint from muscleCrossedJointSet
        if nDOF == 0:
            continue

        # calculating effective dof for that joint
        # effect_DOF = nDOF
        for n_coord in range(nDOF):
            curr_coord = osimModel.getJointSet().get(curr_joint).get_coordinates(n_coord)
            curr_coord_name = curr_coord.getName()

            # skip dof if locked
            if curr_coord.getLocked(currentState):
                continue

            # if coordinate is constrained then the independent coordinate and
            # associated joint will be listed in the sampling "map"
            if curr_coord.isConstrained(currentState):
                constraint_coord_name = curr_coord_name
                # finding the independent coordinate
                [ind_coord_name, ind_coord_joint_name] = getIndipCoordAndJoint(osimModel, constraint_coord_name)
                # updating the coordinate name to be saved in the list
                curr_coord_name = ind_coord_name

                # skip dof if independent coordinate locked (the coord
                # correspondent to the name needs to be extracted)
                if osimModel.getCoordinateSet().get(curr_coord_name).getLocked(currentState):
                    continue

            # ignoring constrained dof if the coordinate that has already
            # been stored
            if osimModel.getCoordinateSet().getIndex(curr_coord_name) in DOF_Index:
                continue
            # NB: DOF_Index is used later in the string generated code.
            # CRUCIAL: the index of dof now is model based ("global") and
            # different from the joint based used until now.
            DOF_Index.append(osimModel.getCoordinateSet().getIndex(curr_coord_name))


    #setting up for loops in order to explore all the possible combination of
    #joint angles (looping on all the dofs of each joint for all the joint
    #crossed by the muscle).
    #The model pose is updated via: " coordToUpd.setValue(currentState,setAngleDof)"
    #The right dof to update is chosen via: "coordToUpd = osimModel.getCoordinateSet.get(n_instr)"
    DOF_Joints = []
    
    for n_instr in range(len(DOF_Index)):
        # print('Evaluating joint -- '+osimModel.getCoordinateSet().get(DOF_Index[n_instr]).getName() + \
        # " -- for a range between "+str(CoordinateBoundaries[n_instr]))
        DOF_Joints.append(osimModel.getCoordinateSet().get(DOF_Index[n_instr]).getName())

    return DOF_Joints

    
### get the groups of coupling DoFs of a osimModel muscle
def getCouplingJoints(osimModel, osim_muscle_model, joints):
    """
    Check if coordinates are coupling when calculating moment arms
    """
    
    if type(joints) != list:
        joints = [joints]
    #initialize the model
    currentState = osimModel.initSystem()
    joints_idx = []
    ang_ranges=[]
    joint_CPset = []
            
    for joint in joints:
        curr_joint = osimModel.getCoordinateSet().get(joint)
        jointRange1 = curr_joint.getRangeMin()
        jointRange2 = curr_joint.getRangeMax()
        
        # assume no joint can rotate more than 180 or -180.
        if jointRange1 < - np.pi:
            jointRange1 = - np.pi
        if jointRange2 > np.pi:
            jointRange2 = np.pi
    
        joints_idx.append(curr_joint)
        ang_ranges.append(np.array([jointRange1,jointRange2]))
        
        joint_CPset.append([joint])
        
    for it in range(0, len(ang_ranges)):
        
        # set minimal angle value for each joint
        for i, ia_range in enumerate(ang_ranges):
            coordToUpd = osimModel.getCoordinateSet().get(joints[i])
            coordToUpd.setValue(currentState, ia_range[0])
        
        # calculate moment arm at current minimal joint angle values
        ma1 = getMomentArmAtJoints(osim_muscle_model, currentState, joints_idx)
        
        # change the it-th joint angle value to maximum & get moment arms
        coordToUpd = osimModel.getCoordinateSet().get(joints[it])
        coordToUpd.setValue(currentState, ang_ranges[it][1])
        
        ma2 = getMomentArmAtJoints(osim_muscle_model, currentState, joints_idx)
        
        for j, sg in enumerate(np.isclose(ma1, ma2, rtol = 1e-4)): # if the MA are with in 0.01%, assume no difference.
        
        # find out the moment arm changes at all other joints by setting
        # maximum angle of it-th joints. It means the it-th joint are coupling
        # with them            
            if not sg and j != it:  
                joint_CPset[j].append(joints[it])
        
    return joint_CPset

### get motion ranges of given coordinates
def getJointRanges_dict(osimModel, joints):
    """
    Extract motion range of given joint coordinates from Osim model
    """
    
    if type(joints) != list:
        joints = [joints]
    #initialize the model
    ang_ranges = {}
            
    for joint in joints:
        curr_joint = osimModel.getCoordinateSet().get(joint)
        jointRange1 = curr_joint.getRangeMin()
        jointRange2 = curr_joint.getRangeMax()
        
        # assume no joint can rotate more than 180 or -180.
        if ('_tx' in joint) or ('_ty' in joint) or ('_tz' in joint):
            []# if joints are translational, do not constrain
        else:
            if jointRange1 < - np.pi:
                jointRange1 = - np.pi
            if jointRange2 > np.pi:
                jointRange2 = np.pi
    
        ang_ranges[joint] = np.array([round(jointRange1, 4), round(jointRange2, 4)])
        
    return ang_ranges

### get motion ranges of given coordinates
def getJointRanges_array(osimModel, joints):
    """
    Extract motion range of given joint coordinates from Osim model
    """
    
    if type(joints) != list:
        joints = [joints]
    #initialize the model
    ang_ranges = {}
            
    for joint in joints:
        curr_joint = osimModel.getCoordinateSet().get(joint)
        jointRange1 = curr_joint.getRangeMin()
        jointRange2 = curr_joint.getRangeMax()
        
        # assume no joint can rotate more than 180 or -180.
        if ('_tx' in joint) or ('_ty' in joint) or ('_tz' in joint):
            []# if joints are translational, do not constrain
        else:
            if jointRange1 < - np.pi:
                jointRange1 = - np.pi
            if jointRange2 > np.pi:
                jointRange2 = np.pi
    
    return np.array([round(jointRange1, 4), round(jointRange2, 4)])

### get motion ranges of all coordinates
def getAllJointsRanges(osimModel): 
    """
    Extract motion range of all joint coordinates of the Osim model
    """
    
    ang_ranges = {}
            
    for i in range(osimModel.getCoordinateSet().getSize()):
        curr_joint = osimModel.getCoordinateSet().get(i)
        jointRange1 = curr_joint.getRangeMin()
        jointRange2 = curr_joint.getRangeMax()
        
        # assume no joint can rotate more than 180 or -180.
        if jointRange1 < - np.pi:
            jointRange1 = - np.pi
        if jointRange2 > np.pi:
            jointRange2 = np.pi
    
        ang_ranges[curr_joint.getName()] = np.array([jointRange1,jointRange2])

    return ang_ranges
    

def getJointsSpannedByMuscle(osimModel, OSMuscleName):
    """
    Given as INPUT a muscle OSMuscleName from an OpenSim model, this function
    returns the OUTPUT structure jointNameSet containing the OpenSim jointNames
    crossed by the OSMuscle.

    It works through the following steps:
    1) extracts the GeometryPath
    2) loops through the sinccgle points, determining the body they belong to
    3) stores the bodies to which the muscle points are attached to
    4) determines the nr of joints based on body indexes
    5) stores the crossed OpenSim joints in the output structure named jointNameSet

    NB this function return the crossed joints independently on the
    constraints applied to the coordinates. Eg patello-femoral is considered as a
    joint, although in Arnold's model it does not have independent
    coordinates, but it is moved in dependency of the knee flexion angle.
    """
    # %useful initializations
    BodySet = osimModel.getBodySet()
    # import ipdb; ipdb.set_trace()
    muscle  = osimModel.getMuscles().get(OSMuscleName)


    jointStructure = getModelJointDefinitions(osimModel)

    #Extracting the PathPointSet via GeometryPath
    musclePath = muscle.getGeometryPath()
    musclePathPointSet = musclePath.getPathPointSet()

    #for loops to get the attachment bodies
    n_body = 1
    jointNameSet = []
    muscleAttachBodies = []
    muscleAttachIndex = []
    muscleAttach=[]
    for n_point in range(musclePathPointSet.getSize()):

    # get the current muscle point
        currentAttachBody = musclePathPointSet.get(n_point).getBodyName()

    # %Initialize
        if n_point == 0:
            previousAttachBody = currentAttachBody
            muscleAttach.append([currentAttachBody, BodySet.getIndex(currentAttachBody)])

    # building a vectors of the bodies attached to the muscles
        if not(currentAttachBody == previousAttachBody):
            muscleAttach.append([currentAttachBody, BodySet.getIndex(currentAttachBody)])
            previousAttachBody = currentAttachBody

    #From distal body checking the joint names going up until the desired
    #OSJointName is found or the proximal body is reached as parent body.
    
    # #check if the muscle attachement are Proximal -> Distal or reversed 
    # first_Body = getParentBodyJoint(jointStructure, muscleAttach[-1][0])
    # second_Body = getParentBodyJoint(jointStructure, muscleAttach[0][0])
    # if first_Body[1] and second_Body[1] and (first_Body[1] < second_Body[1]):
    #     DistalBodyName  = muscleAttach[0][0]
    #     ProximalBodyName= muscleAttach[-1][0]
    # else:
    #     DistalBodyName  = muscleAttach[-1][0]
    #     ProximalBodyName= muscleAttach[0][0]

    # A new way of checking muscle attachment bodies
    # Distal means the body is far away from root body
    # Proximal means the body is closer to root body

    if muscleAttach[0][1] == -1:  # -1 means ground, the root
         DistalBodyName  = muscleAttach[-1][0]
         ProximalBodyName= muscleAttach[0][0]

    elif muscleAttach[-1][1] == -1:  # -1 means ground, the root
        DistalBodyName  = muscleAttach[0][0]
        ProximalBodyName= muscleAttach[-1][0]

    # if the first body index is smaller than second body index
    # then distal = second; proximal = first
    elif muscleAttach[0][1] < muscleAttach[-1][1]:  
        DistalBodyName  = muscleAttach[-1][0]
        ProximalBodyName= muscleAttach[0][0]

    else:
        DistalBodyName  = muscleAttach[0][0]
        ProximalBodyName= muscleAttach[-1][0]

    # start from distal body, moving upward to proximal
    bodyName = DistalBodyName  

    spannedJointNameOld = ''
    NoDofjointNameSet = []
    jointNameSet = []
    
    while (bodyName != ProximalBodyName and bodyName != 'ground'):
        spannedJointName = getChildBodyJoint(jointStructure, bodyName)
        spannedJoint = osimModel.getJointSet().get(spannedJointName)

        if spannedJointName != spannedJointNameOld:

            # bodyName = jointStructure[spannedJointName]['parentFrame']

            # spannedJointNameOld = spannedJointName
        # else:
            if spannedJoint.numCoordinates() != 0:
                jointNameSet.append(spannedJointName)
            else:
                NoDofjointNameSet.append(spannedJointName)

        spannedJointNameOld = spannedJointName
        bodyName = jointStructure[spannedJointName]['parentFrame']

    if not jointNameSet:
        print(['No joint detected for muscle ',OSMuscleName])

    if NoDofjointNameSet:
        for n_v in NoDofjointNameSet:
            print(['Joint ',n_v,' has no dof.'])

    return jointNameSet, NoDofjointNameSet


#### Python porting from https://github.com/modenaxe/MuscleParamOptimizer/blob/master/MATLAB_tool/MuscleParOptTool/private/getIndipCoordAndJoint.m
def getIndipCoordAndJoint(osimModel, constraint_coord_name):
    """
    Function that given a dependent coordinate finds the independent
    coordinate and the associated joint. The function assumes that the
    constraint is a CoordinateCoupleConstraint as used by Arnold, Delp and LLLM.
    The function can be useful to manage the patellar joint for instance.
    """

    # init state
    s = osimModel.initSystem()

    # get coordinate
    constraint_coord = osimModel.getCoordinateSet().get(constraint_coord_name)

    #  double check: if not constrained then function returns
    if not constraint_coord.isConstrained(s):
        print(f"{constraint_coord.getName} is not a constrained coordinate.")
        return ([],[])

    # otherwise search through the constraints
    for n in range(osimModel.getConstraintSet().getSize()):

    # get current constraint
        curr_constr = osimModel.getConstraintSet().get(n)

    # this function assumes that the constraint will be a coordinate
    # coupler contraint ( Arnold's model and LLLM uses this)
    # cast down constraint
        curr_constr_casted = opensim.CoordinateCouplerConstraint.safeDownCast(curr_constr)

    # get dep coordinate and check if it is the coord of interest
        dep_coord_name = curr_constr_casted.getDependentCoordinateName()

        if constraint_coord_name == dep_coord_name:
    # if curr_constr_casted.getIndependentCoordinateNames().getSize
            ind_coord_name_set = curr_constr_casted.getIndependentCoordinateNames()

    # extract independent coordinate and independent joint to which the
    # coordinate refers
            if ind_coord_name_set.getSize() == 1:
                ind_coord_name = curr_constr_casted.getIndependentCoordinateNames().get(0)
                ind_coord_joint_name = osimModel.getCoordinateSet().get(ind_coord_name).getJoint().getName()
    #  return ([],[])
            elif ind_coord_name_set.getSize>1:
                print('getIndipCoordAndJoint.m. The CoordinateCouplerConstraint has more than one indipendent coordinate and this is not managed by this function yet.')

    return [ind_coord_name, ind_coord_joint_name]

### needs to be checked based on https://github.com/modenaxe/MuscleParamOptimizer/blob/master/MATLAB_tool/MuscleParOptTool/getModelJointDefinitions.m
def getModelJointDefinitions(osimModel):
    """
    functon to retun a strcuture for a specifc model, returning a list of the
    joints present in the model and their associated frames
    """

    #first get the associated jointset
    modelJointSet = osimModel.getJointSet()
    #return the number of joints
    numJoints = modelJointSet.getSize()

    # print(f"NumJoints {numJoints}")

    #create an empty structure to hold the data
    jointStructure={}
    for j in range(numJoints):
        tempJoint = modelJointSet.get(j)
        jointStructure[tempJoint.getName()]={}
        jointStructure[tempJoint.getName()]={'childFrame': tempJoint.getChildFrame().getName().replace('_offset',''),'parentFrame': tempJoint.getParentFrame().getName().replace('_offset','')}
    return jointStructure


def getChildBodyJoint(jointStructure, bodyName):

    # return a list of the joints
    allJoints = jointStructure.keys()
    #loop through joints to check the child body entry and return if true
    #NOTE: inefficient but will work temp.

    for joint in allJoints:
        if bodyName == jointStructure[joint]['childFrame']:
            return joint
    print("ERROR!!! NO JOINT FOUND")

def getParentBodyJoint(jointStructure, bodyName):

    # return a list of the joints
    allJoints = jointStructure.keys()
    #loop through joints to check the child body entry and return if true
    #NOTE: inefficient but will work temp.

    for pos, joint in enumerate(allJoints):
        if bodyName == jointStructure[joint]['parentFrame']:
            return joint
    print("ERROR!!! NO JOINT FOUND")

# def getParentBodyJoint(jointStructure, bodyName):

#     # return a list of the joints
#     allJoints = jointStructure.keys()
#     #loop through joints to check the child body entry and return if true
#     #NOTE: inefficient but will work temp.

#     for pos, joint in enumerate(allJoints):
#         if bodyName == jointStructure[joint]['parentFrame']:
#             return (joint, pos)
#     return (None,None)


def getMuscleCoordinates(model, state, muscleName):
    # %% Muscle coordinate finder
    # %   Returns a structure containing the coordinates that a muscle crosses and the
    # %   range of values for which the muscle can generate a moment. This is done by
    # %   examining the moment arm of the muscle across all coordinates in the model
    # %   and recording where the moment arm is nonzero.


    # Get a reference to the concrete muscle class.
    force = model.getMuscles().get(muscleName)
    # muscleClass = force.getConcreteClassName()
    muscle = opensim.Millard2012EquilibriumMuscle.safeDownCast(force)
    # muscle = opensim.Thelen2003Muscle.safeDownCast(force)
    # to be replaced with the function below to make it more general
    # exec('muscle = opensim.'+ muscleClass +'.safeDownCast(force)')
    print(muscleName)

    # Initialize.
    nCoord = model.getCoordinateSet().getSize()
    muscCoord = []  # For storing coordinate values.

    # %% Iterate through coordinates, finding nonzero moment arms.
    for k in range(nCoord):
    #  Get a reference to a coordinate.
        aCoord = model.getCoordinateSet().get(k)
    #  Get coordinate's max and min values.
        rMax = aCoord.getRangeMax()
        rMin = aCoord.getRangeMin()
        rDefault = aCoord.getDefaultValue()
    #  Define three points in the range to test the moment arm.
        totalRange = rMax - rMin
        p = np.zeros(3,)
        p[0] = rMin + totalRange/2
        p[1] = rMin + totalRange/3
        p[2] = rMin + 2*(totalRange/3)

        for i in range(3):
            aCoord.setValue(state, p[i])

    #   Compute the moment arm of the muscle for this coordinate.

            momentArm = muscle.computeMomentArm(state, aCoord)

    #   Avoid false positives due to roundoff e
            tol = 1e-6
            if ( abs(momentArm) > tol ):
                muscCoord.append(k)
                continue
                print("Not Interrupted")

    # % Set the coordinate back to its default value.
        aCoord.setValue(state, rDefault)


    # %% Initialize the structure that will be returned.
    muscle = {}
    muscle['name'] = muscleName
    muscle['coordinates']={}

    # %% Cycle through each coordinate found above and save its range of values. These
    # % will get used later to calculate muscle forces.
    for u in range(len(muscCoord)):
        # Get a reference to the coordinate.
        aCoord = model.getCoordinateSet().get(muscCoord[u])

        # print(aCoord.getName(),aCoord.isConstrained(state),aCoord.getLocked(state),aCoord.getRangeMin(),aCoord.getRangeMax())

        # % Create an array of radian values for the range.
        if aCoord.getRangeMin() < -np.pi:
            RangeMin = -np.pi
        else:
            RangeMin = aCoord.getRangeMin()

        if aCoord.getRangeMax() > np.pi:
            RangeMax = np.pi
        else:
            RangeMax = aCoord.getRangeMax()

        coordRange = np.arange(RangeMin, RangeMax, 0.1)#0.01)
        #  print(aCoord.getName(),RangeMin, RangeMax)
        # % Store the coordinate and its range of values in the structure.
        # eval(['muscle.coordinates.', ...
        # char(model.getCoordinateSet().get(muscCoord(u))), ' = [coordRange];']);
        muscle['coordinates'][aCoord.getName()] = coordRange

    return muscle


def getWrappingSide(osimModel, OSMuscleName, currentState):
    """
    Calculate the position of the wrapping side
    
    Parameters:
        osimModel: Opensim model
        OSMuscleName (str): muscle name
        currentState : State of the Opensim model

    returns:
        wrapMuscleDic: Dictionary with the wrapping center, dimension, type and position of the side

    """
    muscle  = osimModel.getMuscles().get(OSMuscleName)
    musclePath = muscle.getGeometryPath()
    currentPath = musclePath.getCurrentPath(currentState)
    ws = musclePath.getWrapSet()

    wrapMuscleDic={}
    wrapMuscleDicIntersect={}
    for n_ws in range(ws.getSize()):
        w = ws.get(n_ws)
        wo = ws.get(n_ws).getWrapObject()
        wrapMuscleDic[w.get_wrap_object()] = {"center":np.array([wo.get_translation()[i] for i in range(3)]),
                                        "dimension":wo.getDimensionsString(),
                                        "type":wo.getWrapTypeName()}
        wrapMuscleDicIntersect[w.get_wrap_object()] = []
        # Heuristic based on the assumtion that 
        # 1. last muscle attachment and wrapping surface are in the same relative coordinate frame
        # 2. the muscle attachment is already on the side of the wrapping we want to enfoce
        if w.get_range(0)>0 and w.get_range(1) > 0:
            cp = currentPath.get(w.get_range(0))
            mp = np.array([cp.getLocation(currentState).get(i) for i in range(3)])
            wo = ws.get(n_ws).getWrapObject()
            cw = np.array([wo.get_translation().get(i) for i in range(3)])
            side = mp+3*(mp-cw)
            wrapMuscleDic[w.get_wrap_object()].update({"side_pos":np.round(side,4)})

    def calculateNewPos(center, pos1, pos2, mag=3):
        """ 
            Function to triangulate the position of the side of the wrapping
            based on the intersaction of the muscle with the wrapping surface 
        """
        x0, y0, z0 = center
        x1, y1, z1 = pos1
        x2, y2, z2 = pos2

        xm, ym, zm = (x1+x2)/2, (y1+y2)/2, (z1+z2)/2

        xd, yd, zd = (xm-x0)*mag+x0, (ym-y0)*mag+y0, (zm-z0)*mag+z0

        return xd, yd, zd


    for i in range(currentPath.getSize()):
        cp = currentPath.get(i)
        p3d = np.array([cp.getLocation(currentState)[i] for i in range(3)])

        if cp.getWrapObject():
            wo = cp.getWrapObject()
            wrapMuscleDicIntersect[wo.getName()].append(p3d)
            
    for s_wo in wrapMuscleDicIntersect:
        if wrapMuscleDicIntersect[s_wo]:
            if wrapMuscleDic[s_wo]['type'] == 'torus':
                wrapMuscleDic[s_wo].update({"side_pos":np.round(wrapMuscleDic[s_wo]['center'],4)})
            else:
                xd, yd, zd = calculateNewPos(wrapMuscleDic[s_wo]['center'],wrapMuscleDicIntersect[s_wo][0],wrapMuscleDicIntersect[s_wo][1])
                wrapMuscleDic[s_wo].update({"side_pos":np.round([xd, yd, zd],4)})
            # print(s_wo, wrapMuscleDic[s_wo]['type'])
    return wrapMuscleDic

def getAllIndependentCoordinates(osimModel):
    """
    Get all indpendent coordinates in an Osim model
    """
    
    # load coordinates and initialize model state
    coordSet = osimModel.getCoordinateSet()
    state = osimModel.initSystem()
    
    DOF_Index = []
    joint_names = []
    
    for curr_coord in coordSet:
        
        curr_coord_name = curr_coord.getName()
        
        # find the independed joints if the current coordinate is contrainted
        if curr_coord.isConstrained(state) and not curr_coord.getLocked(state):
            
            constraint_coord_name = curr_coord_name
            # finding the independent coordinate
            [ind_coord_name, ind_coord_joint_name] = getIndipCoordAndJoint(osimModel, constraint_coord_name)
            # updating the coordinate name to be saved in the list
            curr_coord_name = ind_coord_name
            
            
        # skip dof if independent coordinate locked (the coord
        # correspondent to the name needs to be extracted)
        if osimModel.getCoordinateSet().get(curr_coord_name).getLocked(state):
            continue
    
        # if curr_coord_name is already in the DoF_index, then skip saving    
        if osimModel.getCoordinateSet().getIndex(curr_coord_name) in DOF_Index:
            continue
        
        # NB: DOF_Index is used later in the string generated code.
        # CRUCIAL: the index of dof now is model based ("global") and
        # different from the joint based used until now.
        DOF_Index.append(osimModel.getCoordinateSet().getIndex(curr_coord_name))
        joint_names.append(curr_coord_name)
        
    return joint_names, DOF_Index

def extractMarkerSet(osimModel_path):
    """
        Extract marker list from Osim model for forward kinematic check
    """
    
    osimModel = opensim.Model(osimModel_path)
    
    markerSet = osimModel.getMarkerSet()
    markerNames = []

    for marker in markerSet:
        markerNames.append(marker.getName())
        
    return markerNames
    

def calculateEndPoints_osim(osimModel_path, endPoints, N_EvalPoints):
    """
    Calculate the positions of endpoints when iterating all joint angle meshes.
    This is to check the Geometry and Joint Definition matches between the Opensim
    and the converted MuJoCo models.
    Parameters
    ----------
    osimModel_path : string
        Path and name of the Osim model that will be evaluated.
    endPoints : list of string
        The names of the endpoints whose positions will be evaluated.
    Returns
    -------
    evaData: dict
        The evaluation data, including the joint angle meshes and the corresponding
        endpoint locations
    """
    
    osimModel = opensim.Model(osimModel_path)
    
    coordSet = osimModel.getCoordinateSet()
    state = osimModel.initSystem()
        
    markerSet = osimModel.getMarkerSet()
        
    # find index of the targetting markers inside the marker set
    end_id = []
    for endPoint in endPoints:
        end_id.append(markerSet.getIndex(endPoint))
    
    # get the joints and range informatino
    joint_names, DOF_Index = getAllIndependentCoordinates(osimModel)
    
    ang_ranges = getJointRanges_dict(osimModel, joint_names)
    
    # Storing the location of end points
    endPos =[]
    
    # as musculoskeletal model getting complex (the number of joints increases)
    #, it is harder to run a batch loop of all possible postures. Therefore, 
    # N_EvalPoints random postures were selected to do the comparison.
    
    for n_eval in range(N_EvalPoints):
        
        # set up coordinate values
        for idof, dof in enumerate(DOF_Index):
            
            range_ind = ang_ranges[joint_names[idof]]
            
            # generate the random joint angles based the n th evaluation point 
            np.random.seed(n_eval)
            jointEval = np.array(range_ind[0]) + 0.8*np.random.random(1)* \
                    (np.array(range_ind[1]) - np.array(range_ind[0]))
            
            coordSet.get(dof).setValue(state, jointEval[0])
            coordSet.get(dof).setSpeedValue(state, 0)
    
        # extract the marker location after the simulation
        endiPos = []
        for ed in end_id:
            endiiPos = []
            for l in range(markerSet.get(ed).getLocationInGround(state).size()):
                endiiPos.append(markerSet.get(ed).getLocationInGround(state).__getitem__(l))
            endiPos.append(endiiPos)
            
        endPos.append(endiPos)
        
    return ang_ranges, endPos

# update the osim model coordinate variable values
def updOsimCoordEndPoints(osimModel, DOF_Index, jointEval, endPoints):
    
    state = osimModel.initSystem()
    coordSet = osimModel.getCoordinateSet()
    
    markerSet = osimModel.getMarkerSet()
        
    # find index of the targetting markers inside the marker set
    end_id = []
    for endPoint in endPoints:
        end_id.append(markerSet.getIndex(endPoint))
    
    stepsize = 0.001
    # initialize simulation    
    # manager = opensim.Manager(osimModel)
    # manager.initialize(state)
    
    # run simulation one time before setting up the corrdinates, to makesure the
    # simulation results for markers are correct.
    state.setTime(0)
    # state = manager.integrate(stepsize)
    
    # set up coordinate values
    for idof, dof in enumerate(DOF_Index):
        
        coordSet.get(dof).setValue(state, jointEval[idof])
        coordSet.get(dof).setSpeedValue(state, 0)
        
    # initialize simulation    
    manager = opensim.Manager(osimModel)
    # start simulation
    # state.setTime(n_eval*stepsize)
    
    # initilize the manager
    manager.initialize(state)
        
    # do the intergration
    state = manager.integrate(stepsize)
    
    # extract the marker location after the simulation
    endiPos = []
    for ed in end_id:
        endiiPos = []
        for l in range(markerSet.get(ed).getLocationInGround(state).size()):
            endiiPos.append(markerSet.get(ed).getLocationInGround(state).__getitem__(l))
        endiPos.append(endiiPos)
    
    return endiPos

def getMomentArmAtJoints(muscleModel, state, joint_set):
    """
    Compute moment arm of a muscle at a joint that it wraps.
    """

    mom_arm = []
    for joint in joint_set:
        mom_arm.append(muscleModel.computeMomentArm(state, joint))
        
    return np.asarray(mom_arm)

def computeMomentArm(osimModel, osim_muscle_model, joints, ang_ranges, N_EvalPoints):
    if type(joints) != list:
        joints = [joints]
    #initialize the model
    currentState = osimModel.initSystem()
    joints_idx = []
    ang_mesh=[]
    
    for ij, joint in enumerate(joints):
        curr_joint = osimModel.getCoordinateSet().get(joint)
        joints_idx.append(curr_joint)
        ang_mesh.append(np.linspace(ang_ranges[ij][0], ang_ranges[ij][1], N_EvalPoints))

    mom_arm =[]
    for setAngleDofs in itertools.product(*ang_mesh):
        # this itertools.product will generate the angle mesh list in this structure:
        # [Ang11, Ang21, Ang31], [Ang11, Ang21, Ang32], ...
        # [Ang11, Ang22, Ang31], [Ang11, Ang22, Ang32], ...
        # ...
        # [Ang12, Ang21, Ang31], [Ang12, Ang21, Ang32], ...
        # ...
        # ...

        for i,ia in enumerate(setAngleDofs):
            # coordToUpd = osimModel.getCoordinateSet().get(joints[i])
            joints_idx[i].setValue(currentState,ia)
        
        mom_arm.append(getMomentArmAtJoints(osim_muscle_model, currentState, joints_idx))
    
    return np.array(mom_arm)

# update the osim model coordinate variable values
def updOsimCoordMomentArm(osimModel, osim_muscle_model, joints_idx, DOF_index, jointEval):
    
    state = osimModel.initSystem()
    coordSet = osimModel.getCoordinateSet()
    
    
    stepsize = 0.001
    # initialize simulation    
    # manager = opensim.Manager(osimModel)
    # manager.initialize(state)
    
    # run simulation one time before setting up the corrdinates, to makesure the
    # simulation results for markers are correct.
    state.setTime(0)
    # state = manager.integrate(stepsize)

    # set up coordinate values
    for idof, dof in enumerate(DOF_index):
        
        coordSet.get(dof).setValue(state, jointEval[idof])
        coordSet.get(dof).setSpeedValue(state, 0)
        
    # initialize simulation    
    manager = opensim.Manager(osimModel)
    # start simulation
    # state.setTime(n_eval*stepsize)
    
    # initilize the manager
    manager.initialize(state)
        
    # do the intergration
    state = manager.integrate(stepsize)

    return getMomentArmAtJoints(osim_muscle_model, state, joints_idx)


def getMuscleLengthList(osimModel, osim_muscle, joints, ang_ranges, speedy):
    # get the a list of roughly distributed MTU lengths and forcces inside the joint angle ranges
    
    if type(joints) != list:
        joints = [joints]
        
    #initialize the model, then calcualted the joint ranges
    currentState = osimModel.initSystem()
    joints_idx = []
    ang_mesh = []

    if speedy:
        # contraint the max evaluate data points based on the number of unique joints
        if len(joints) == 1:
            evalN_jnt = 25
        elif len(joints) == 2:
            evalN_jnt = 7
        elif len(joints) == 3:
            evalN_jnt = 4
        elif len(joints) == 4:
            evalN_jnt = 3
        else:
            evalN_jnt = 2
            
        N_eval = 7  # number of muscle length check nodes

    else:
        # contraint the max evaluate data points based on the number of unique joints
        if len(joints) == 1:
            evalN_jnt = 25
        elif len(joints) == 2:
            evalN_jnt = 11
        elif len(joints) == 3:
            evalN_jnt = 7
        elif len(joints) == 4:
            evalN_jnt = 5
        elif len(joints) == 5:
            evalN_jnt = 4
        else:
            evalN_jnt = 3
            
        N_eval = 15  # number of muscle length check nodes

    for ij, joint in enumerate(joints):
        curr_joint = osimModel.getCoordinateSet().get(joint)
        joints_idx.append(curr_joint)
        ang_mesh.append(np.linspace(ang_ranges[ij][0], ang_ranges[ij][1], evalN_jnt))

    # calculate muscle tendon lengths based on the meshes of joint angles
    mtu_len = []
    jit_list = []
    for setAngleDofs in itertools.product(*ang_mesh):
        for i,ia in enumerate(setAngleDofs):
            # coordToUpd = osimModel.getCoordinateSet().get(joints[i])
            joints_idx[i].setValue(currentState, ia)
            
        jit_list.append(setAngleDofs)
        mtu_len.append(osim_muscle.getGeometryPath().getLength(currentState))
        
    # generate the even distributed muscle tendon lenghts based on the min 
    # and max of muscle lengths that calculated from the mesh joint angles
    mtu_len_dis = np.linspace(min(mtu_len), max(mtu_len), N_eval)
    
    # find the closed muscle tendon lengths from the even distributed mtu lengths
    mtu_len_set = []
    jit_list_set = []
    for mtu_len_i in mtu_len_dis:
        mtu_len_dis_ind = min(range(len(mtu_len)), key=lambda i: abs(mtu_len[i]-mtu_len_i))
        
        mtu_len_set.append(mtu_len[mtu_len_dis_ind])
        jit_list_set.append(jit_list[mtu_len_dis_ind])
    
    return np.array(mtu_len_set), jit_list_set


def getMuscleForceMaps(osimModel, osim_muscle, joints, jit_list_set, act_list):
    # get muscle length force maps
    
    #initialize the model, then calcualted the joint ranges
    currentState = osimModel.initSystem()
    
    # then calculate the equilibrated muscle force from the selected joint meshes
    col = len(jit_list_set)
    row = len(act_list)
    mtu_act = np.zeros((row, col))
    mtu_pas = np.zeros((row, col))
    mtu_for = np.zeros((row, col))
    # mtu_ang = []
    for i, setAngleDofs in enumerate(jit_list_set):
        for ii,iia in enumerate(setAngleDofs):
            coordToUpd = osimModel.getCoordinateSet().get(joints[ii])
            coordToUpd.setValue(currentState,iia)
            
        for j, ja in enumerate(act_list):
            osim_muscle.setActivation(currentState, ja)
            osimModel.equilibrateMuscles(currentState)
            # if ja == 1:  # extract active and passive forces only when muscle activation == 1
            mtu_act[j, i] = osim_muscle.getActiveFiberForce(currentState)
            mtu_pas[j, i] = osim_muscle.getPassiveFiberForce(currentState)
            mtu_for[j, i] = osim_muscle.getFiberForceAlongTendon(currentState)  # might return Nan, don't know why...

    # if total fiber force contains NaN, then calculate from active and passive forces
    if np.isnan(mtu_for).any():
        return np.array(mtu_act) + np.array(mtu_pas), np.array(mtu_act), np.array(mtu_pas)

    return np.array(mtu_for), np.array(mtu_act), np.array(mtu_pas)


def getMuscleProperties(osim_muscle):
    # This function is to get the muscle properties, such as activation dynamics
    # optimal fiber lengths, fiber/tendon slack lengths, and so on.
    
    # current code only works for the Millard muscle type
    
    # Millard_muscle = opensim.Millard2012EquilibriumMuscle.safeDownCast(osim_muscle)
    
    fiber_opt = osim_muscle.get_optimal_fiber_length()
    tendon_sla = osim_muscle.get_tendon_slack_length()
    penna_opt = osim_muscle.get_pennation_angle_at_optimal()
    fmax = osim_muscle.get_max_isometric_force()
    # FVC  = osim_muscle.getForceVelocityCurve()
    # fvmax = FVC.getMaxEccentricVelocityForceMultiplier()

    # min_a = osim_muscle.getMinControl()
    # max_a = osim_muscle.getMaxControl()
    # t_act = Millard_muscle.get_activation_time_constant()
    # t_det = Millard_muscle.get_deactivation_time_constant()
    
    muscle_property_dict = {'fiber_opt': fiber_opt, 'tendon_sla': tendon_sla, 
                            'penna_opt': penna_opt, 'fmax': fmax}
    
    return muscle_property_dict
