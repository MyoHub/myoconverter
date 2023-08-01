import numpy as np
from numpy import sin, cos, sqrt, arctan

# This utilis cotains the coordinate convertion and rotation functions

def spherical2cartesian(wrap_centre, site_pose_sph):
    """
    Transfer the site_pose in the Spherical coordinate into Catersian coordinate,
    relative to the wrap_centre (for sphere objects)

    Parameters:
        wrap_centre: the centre location of the wrapping object, in Catersian coordinate
        site_pose_sph: the position of the site point, in Spherical coordinate:
            r: float, ra [0, inf]
            theta: float, vectical angle [0, pi]
            phi: float, horizental angle [-pi, pi]
    """
    x = site_pose_sph[0]*cos(site_pose_sph[2])*sin(site_pose_sph[1]) + wrap_centre[0]  # x = r*cos(\phi)*sin(\theta)
    y = site_pose_sph[0]*sin(site_pose_sph[2])*sin(site_pose_sph[1]) + wrap_centre[1]  # y = r*sin(\phi)*sin(\theta)
    z = site_pose_sph[0]*cos(site_pose_sph[1]) + wrap_centre[2]  # z = r*cos(\theta)

    return np.array([x, y, z])


def cartesian2spherical(wrap_centre, site_pose):
    """
    Transfer the site_pose in the Catersian coordinate into Spherical coordinate,
    relative to the wrap_centre (for sphere objects)

    Parameters:
        wrap_centre: the centre location of the wrapping object, in Catersian coordinate
        site_pose: the position of the site point, in Catersian coordinate
        
    Outputs:
        r: float, [0, inf], radius of the sphere
        theta: float, [0 pi], rotation angle with respect to the z axis
        phi: float, [-pi, pi],  rotation angle in the x-y plane
        
    """

    # calculate the radius first
    r = np.sum((site_pose - wrap_centre)**2)

    if r != 0:   # calculate theta and phi, if r != 0
        theta = np.arctan2(np.sqrt(np.sum((site_pose[0:2] - wrap_centre[0:2])**2)),\
                           site_pose[2]-wrap_centre[2])  # \theta = arctan2(y, x)
        phi = np.arctan2(site_pose[1]-wrap_centre[1], site_pose[0]-wrap_centre[0])  # \phi = arctan2(y, x)

    else:   # if r == 0, theta and pi both equal 0
        theta = 0
        phi = 0

    return np.array([r, theta, phi])

def quaternionRotaionInv(unitQuat, site_pose):
    """
    Rotation the site_pose based on the unit quaternion rotation vector.
    
    Parameters:
        unitQuat: unit vector of the quaternion rotation angles
        site_pose: the catesian space location of the site point
    """
    
    # calculate rotation matrix A based on the unit quaternion vector
    a = unitQuat[0]
    b = unitQuat[1]
    c = unitQuat[2]
    d = unitQuat[3]

    # A = np.array([[1 - 2*(qj**2 + qk**2), 2*(qi*qj - qk*qr), 2*(qi*qk + qj*qr)],
    #      [2*(qi*qj + qk*qr), 1 - 2*(qi**2 + qk**2), 2*(qj*qk - qi*qr)],
    #      [2*(qi*qk - qj*qr), 2*(qj*qk + qi*qr), 1 - 2*(qi**2 + qj**2)]])
    
    
    A = np.array([[a**2 + b**2 - c**2 - d**2, 2*b*c - 2*a*d, 2*b*d + 2*a*c],
                  [2*b*c + 2*a*d, a**2 - b**2 + c**2 - d**2, 2*c*d - 2*a*b],
                  [2*b*d - 2*a*c, 2*c*d + 2*a*b, a**2 - b**2 - c**2 + d**2]])
    
    new_site_pose = np.dot(np.linalg.inv(A), site_pose)  # calculate new pose after rotation.
    
    return new_site_pose

def quaternionRotaion(unitQuat, site_pose):
    """
    Rotation the site_pose based on the unit quaternion rotation vector.
    
    Parameters:
        unitQuat: unit vector of the quaternion rotation angles
        site_pose: the catesian space location of the site point
    """
    
    # calculate rotation matrix A based on the unit quaternion vector
    a = unitQuat[0]
    b = unitQuat[1]
    c = unitQuat[2]
    d = unitQuat[3]

    # A = np.array([[1 - 2*(qj**2 + qk**2), 2*(qi*qj - qk*qr), 2*(qi*qk + qj*qr)],
    #      [2*(qi*qj + qk*qr), 1 - 2*(qi**2 + qk**2), 2*(qj*qk - qi*qr)],
    #      [2*(qi*qk - qj*qr), 2*(qj*qk + qi*qr), 1 - 2*(qi**2 + qj**2)]])
    
    
    A = np.array([[a**2 + b**2 - c**2 - d**2, 2*b*c - 2*a*d, 2*b*d + 2*a*c],
                  [2*b*c + 2*a*d, a**2 - b**2 + c**2 - d**2, 2*c*d - 2*a*b],
                  [2*b*d - 2*a*c, 2*c*d + 2*a*b, a**2 - b**2 - c**2 + d**2]])
    
    new_site_pose = np.dot(A, site_pose)  # calculate new pose after rotation.
    
    return new_site_pose


def cartesian2cylindrical(wrap_centre, site_pose):
    """
    Transfer the site_pose in the Catersian coordinate into Cylindrical coordinate,
    relative to the wrap_centre (for sphere objects)

    Parameters:
        wrap_centre: the centre location of the wrapping object, in Catersian coordinate
        site_pose: the position of the site point, in Catersian coordinate
        
    Outputs:
        rho: float, [0, inf], radius of the cylinder
        phi: float, [-pi, pi], rotation angle
        z: float, [-inf, inf], distance to the origin O, along the longitudinal axis        
    """
    
    # calculate the rho, phi, z values in cylindarical coordinate
    rho = np.sqrt((site_pose[0] - wrap_centre[0])**2 + (site_pose[1] - wrap_centre[1])**2)  # range [0, inf]
    phi = np.arctan2(site_pose[1] - wrap_centre[1], site_pose[0] - wrap_centre[0])  # range [-pi, pi]
    z = site_pose[2] - wrap_centre[2]   # range [-inf, inf]

    return np.array([rho, phi, z])


def cylindarical2cartesian(wrap_centre, site_pose_cyl):
    """
    Transfer the site_pose in the Cylindarical coordinate into Catersian coordinate,
    relative to the wrap_centre (for sphere objects)

    Parameters:
        wrap_centre: the centre location of the wrapping object, in Catersian coordinate
        site_pose_sph: the position of the site point, in Cylindarical coordinate
        
    Outputs:
        x: float, [-inf, inf], x axis location of the local coordinate
        y: float, [-inf, inf], y axis location of the local coordinate
        z: float, [-inf, inf], z axis location of the local coordinate
        
    """
    
    x = site_pose_cyl[0]*cos(site_pose_cyl[1]) + wrap_centre[0]
    y = site_pose_cyl[0]*sin(site_pose_cyl[1]) + wrap_centre[1]
    z = site_pose_cyl[2] + wrap_centre[2]
    
    return np.array([x, y, z])


# def quaternion2euler(unitQuat):
#     """
#     Transfer the united quaternion vector into three euler angles.
#     Followed the conversion equation in Wiki: 
#     https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    
#     Parameters:
#         unitQuat: united quaternion vector
        
#     Outputs:
#         Euler rotation angles:
#             yaw: float, [-pi, pi], yaw angle
#             pitch: float, [-pi/2, pi/2], pitch angle
#             roll: float, [-pi, pi], roll angle
#     """
#     q0 = unitQuat[0]   # cos(theta/2)
#     q1 = unitQuat[1]   # bx*sin(theta/2)
#     q2 = unitQuat[2]   # by*sin(theta/2)
#     q3 = unitQuat[3]   # bz*sin(theta/2)
        
#     roll = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
#     pitch = np.arcsin(2*(q0*q2 - q3*q1))
#     yaw = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2**2 + q3**2))
    
#     return np.array([yaw, pitch, roll])


# def euler2quaternion(eulerAng):
#     """
#     Transfer the euler angles into united quaternion vector.
#     Followed the conversion equation in Wiki: 
#     https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    
#     Parameters:
#         eulerAng: [yaw, pitch, roll]
        
#     Outputs:
#         united quaternion vectors
#     """
#     cy = np.cos(eulerAng[0]/2)
#     sy = np.sin(eulerAng[0]/2)
#     cp = np.cos(eulerAng[1]/2)
#     sp = np.sin(eulerAng[1]/2)
#     cr = np.cos(eulerAng[2]/2)
#     sr = np.sin(eulerAng[2]/2)
        
#     unitQuat = np.array([sr * cp * cy - cr * sp * sy,\
#                         cr * sp * cy + sr * cp * sy,\
#                         cr * cp * sy - sr * sp * cy,\
#                         cr * cp * cy + sr * sp * sy])
    
#     return unitQuat
    


def TaitBryanRotationMatrix(Sequence, angles):
    
    
    a = angles[0]
    b = angles[1]
    r = angles[2]
    
    s1 = sin(a)
    c1 = cos(a)
    s2 = sin(b)
    c2 = cos(b)
    s3 = sin(r)
    c3 = cos(r)    
    
    if Sequence == 'xzy':
        rot = [[cos(b)*cos(r), -sin(b), cos(b)*sin(r)],
               [sin(a)*sin(r) + cos(a)*cos(r)*sin(b), cos(a)*cos(b), cos(a)*sin(b)*sin(r) - cos(r)*sin(a)],
               [cos(r)*sin(a)*sin(b) - cos(a)*sin(r), cos(b)*sin(a), cos(a)*cos(r) + sin(a)*sin(b)*sin(r)]]
        
    elif Sequence == 'yzx':
        rot = [[cos(a)*cos(b), sin(a)*sin(r) - cos(a)*cos(r)*sin(b), cos(r)*sin(a) + c1*s2*s3],
               [s2, c2*c3, -c2*s3],
               [-c2*s1, c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3]]
        
    elif Sequence == 'zxy':
        rot = [[c1*c3 - s1*s2*s3, -c2*s1, c1*s3 + c3*s1*s2],
               [c3*s1 + c1*s2*s3, c1*c2, s1*s3 - c1*c3*s2],
               [-c2*s3, s2, c2*c3]]
        
    elif Sequence == 'zyx':
        rot = [[c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2],
               [c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3],
               [-s2, c2*s3, c2*c3]]
        
    elif Sequence == 'yxz':
        rot = [[c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3, c2*s1],
               [c2*s3, c2*c3, -s2],
               [c1*s2*s3 - c3*s1, c1*c3*s2 + s1*s3, c1*c2]]
        
    elif Sequence == 'xyz': 
        rot = [[cos(b)*cos(r), -cos(b)*sin(r), sin(b)],\
    	      [cos(a)*sin(r) + cos(r)*sin(a)*sin(b), cos(a)*cos(r) - sin(a)*sin(b)*sin(r), -cos(b)*sin(a)],\
    	      [sin(a)*sin(r) - cos(a)*cos(r)*sin(b), cos(r)*sin(a) + cos(a)*sin(b)*sin(r), cos(a)*cos(b)]]
            
    return rot

def euler_change_sequence(oldSequence, old_angles, newSequence):
    """
    From the old sequence euler rotation angles, get euler rotation angles with
    the sequence of x-y-z.
    
    Parameters:
        oldSequence: the old order of sequence of the euler rotation
    	old_angles: three euler angles in the old rotation sequence
    	newSequence: the new order of sequence of the euler rotation
    	
    Outputs:
    	eulerNew: the new euler angles with the new rotation sequence.
    """
    
    rot = TaitBryanRotationMatrix(oldSequence, old_angles)
    	      
    if newSequence == 'xzy':
        a_new = arctan(rot[3][1]/rot[1][1])
        b_new = arctan(-rot[0][1]/sqrt(1-rot[0][1]**2))
        r_new = arctan(rot[0][2]/rot[0][0])
    elif newSequence == 'xyz':
        a_new = arctan(-rot[1][2]/rot[2][2])
        b_new = arctan(rot[0][2]/sqrt(1 - rot[0][2]**2))
        r_new = arctan(-rot[0][1]/rot[0][0])
        
    return [a_new, b_new, r_new]


def euler_change_sequence_bodyRotationFirst(bodySequence, body_angle, oldSequence, old_angles, newSequence):
    """
    From the old sequence euler rotation angles, get euler rotation angles with
    the sequence of x-y-z.
    
    Parameters:
        oldSequence: the old order of sequence of the euler rotation
    	old_angles: three euler angles in the old rotation sequence
    	newSequence: the new order of sequence of the euler rotation
    	
    Outputs:
    	eulerNew: the new euler angles with the new rotation sequence.
    """
    rot1 = TaitBryanRotationMatrix(bodySequence, body_angle)
    
    rot2 = TaitBryanRotationMatrix(oldSequence, old_angles)
    
    rot = np.dot(np.array(rot1), np.array(rot2))
    

    if newSequence == 'xzy':
        a_new = arctan(rot[3][1]/rot[1][1])
        b_new = arctan(-rot[0][1]/sqrt(1-rot[0][1]**2))
        r_new = arctan(rot[0][2]/rot[0][0])
    elif newSequence == 'xyz':
        a_new = arctan(rot[1][2]/rot[2][2])  # negative sign is removed, don't understand why yet!
        b_new = arctan(rot[0][2]/sqrt(1 - rot[0][2]**2))
        r_new = arctan(-rot[0][1]/rot[0][0])
        
    return [a_new, b_new, r_new]
    
    


