import math
from scipy.spatial.transform import Rotation as R 
def euler2quat(euler_pos):
    # tx_, ty_, tz_ = euler_pos[0:3]
    # sx = math.sin(0.5 * tx_)
    # cx = math.cos(0.5 * tx_)
    # sy = math.sin(0.5 * ty_)
    # cy = math.cos(0.5 * ty_)
    # sz = math.sin(0.5 * tz_)
    # cz = math.cos(0.5 * tz_)

    # qx_ = sx * cy * cz + cx * sy * sz
    # qy_ = -sx * cy * sz + cx * sy * cz
    # qz_ = sx * sy * cz + cx * cy * sz
    # qw_ = -sx * sy * sz + cx * cy * cz

    # Q_ = [qx_, qy_, qz_, qw_]
    # return Q_

    r = R.from_euler('zyx', euler_pos, degrees=False)
    return r.as_quat()