import numpy as np
from scipy.spatial.transform import Rotation as R

Origin = np.array([0.750084268750818, -0.396146707532958, 1.293672642992076])
rot = R.from_quat([-0.188348129218565,0.098321518123007, 0.950766413665119, 0.225612694545813])

rot_inv = rot.inv()

Origin_Pose = rot_inv.apply(Origin)

Tag_1 = np.array([-0.029628919951449, 0.813203462309145, 2.257667863250954])

Tag_1_rot = R.from_quat([-0.187620095766046, 0.085041825369440, 0.952734744979311, 0.223300455243112])

Tag_1_rot_inv = Tag_1_rot.inv()

Tag_2_initial_pose = Tag_1_rot_inv.apply(Tag_1)

frame_rotation = Tag_1_rot_inv * rot

Tag_1_Pose = frame_rotation.apply(Tag_1)

Tag_1_Pose = (Tag_1_Pose - Origin_Pose) * 1.62

print(Tag_1_Pose)