import numpy as np
from scipy.spatial.transform import Rotation as R

def q_conjugate(q):
    # Return the conjugate of the quaternion
    return np.array([q[0], -q[1], -q[2], -q[3]])

def q_mult(q1, q2):
    # Multiply two quaternions
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return np.array([w, x, y, z])

def qv_mult(q1, v1):
    # Rotate a vector by a quaternion
    q2 = np.array([0.0] + v1.tolist())
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]



def rotateframe(q, point):
    # Convert point to quaternion with a real part of 0
    uq = np.array([0] + point.tolist())
    
    # Normalize the quaternion q
    qn = R.from_quat(q).as_quat()
    qn = qn / np.linalg.norm(qn)
    
    # Convert normalized quaternion back to Rotation for easy manipulation
    qn_rot = R.from_quat(qn)
    
    # Apply the rotation: vq = q * uq * q_inv
    # For quaternion multiplication, use the rotate method
    vq = qn_rot.apply(uq[1:])
    
    # Return the vector part of the resulting quaternion
    return vq

Origin = np.array([0.750084268750818, -0.396146707532958, 1.293672642992076])
rot = R.from_quat([-0.188348129218565, 0.098321518123007, 0.950766413665119, 0.225612694545813])
rot_Origin = [-0.188348129218565, 0.098321518123007, 0.950766413665119, 0.225612694545813]

Pose_Origin = (0.750084268750818, -0.396146707532958, 1.293672642992076)

Origin_Pose = rot.apply(Origin)

Tag_1 = np.array([-0.029628919951449, 0.813203462309145, 2.257667863250954])

Tag_1_Pose = (-0.029628919951449, 0.813203462309145, 2.257667863250954)

Tag_1_rot = R.from_quat([-0.187620095766046, 0.085041825369440, 0.952734744979311, 0.223300455243112])
rot_2_2 = [-0.187620095766046, 0.085041825369440, 0.952734744979311, 0.223300455243112]

Tag_2_initial_pose = Tag_1_rot.apply(Tag_1)

Tag_2_Pose = (Tag_2_initial_pose - Origin_Pose) * 1.638361

print(Tag_2_Pose)

Tag_2 = rotateframe(Tag_1_rot.as_quat(), Tag_1) - rotateframe(rot.as_quat(), Origin)

Pose_2_2 = qv_mult(rot_2_2, Tag_1_Pose) - qv_mult(rot_Origin, Pose_Origin)
print(Pose_2_2)