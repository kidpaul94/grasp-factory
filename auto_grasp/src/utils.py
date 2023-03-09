import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Pose

class Conversion():
    
    @staticmethod
    def list2pose(sixd: list, degrees: bool = True) -> Pose:
        """
        Convert 6D representation to ROS Pose message format.

        Parameters
        ----------
        sixd : 1x6 : obj : `list`
            list of xyz and three euler angles
        degrees : bool
            whether euler angles are given in degree or radian

        Returns
        -------
        `Pose` : pose message composed with xyz and quaternion
        """
        position = Point(sixd[0], sixd[1], sixd[2])
        temp = R.from_euler('xyz', sixd[3:], degrees).as_quat()
        quat = Quaternion(temp[0], temp[1], temp[2], temp[3])

        return Pose(position=position, orientation=quat)

    @staticmethod
    def cpp2T(center: np.ndarray, direction: np.ndarray, aprv: np.ndarray) -> np.ndarray:
        """
        Convert ROS Pose message format to homegeneous transformation. 

        Parameters
        ----------
        center : 3x1 : obj : `np.ndarray`
            gripper center w.r.t the world frame
        direction : 3x1 : obj : `np.ndarray`
            gripper direction w.r.t the world frame
        aprv : 3xN : obj : `np.ndarray`
            approach vector of a gripper of a cpp

        Returns
        -------
        res : 4x4 : obj : `np.ndarray`
            homegeneous transformation format
        """
        last_ax = np.cross(direction, aprv)
        R = np.eye(3)
        R[0,:], R[1,:], R[2,:] = direction, last_ax, aprv

        res = np.eye(4)
        res[:3,:3] = R
        res[3,:3] = R @ center.reshape((3, 1))
        
        return res
    
def grasp_gen(path: str = None) -> np.ndarray:
    """
    Generate grasp dictionary based on the current object pose and 
    contact point pairs (cpp).

    Parameters
    ----------
    path : string
        root path to a .txt file of cpps and aprv

    Returns
    -------
    centers : 3xN : obj : `np.ndarray`
        array of potential gripper centers w.r.t the world frame
    directions : 3xN : obj : `np.ndarray`
        array of potential gripper directions w.r.t the world frame
    aprvs : 1xN : obj : `list`
        list of approach vectors for each cpp
    """
    path_cpps = f'{path}/cpps.txt'
    path_aprvs = f'{path}/aprvs.txt'

    with open(path_cpps) as f1:
        cpps = eval(f1.read())
    cpps = np.asarray(cpps).T

    with open(path_aprvs) as f2:
        aprvs = eval(f2.read())
    
    # array of potential gripper configurations w.r.t the object frame
    # 0.005 = 0.001(mm to m) * 0.5
    centers = 0.0005 * (cpps[:3,:] + cpps[3:6,:])
    directions = cpps[:3,:] - cpps[3:6,:]
    
    return centers, directions, aprvs
