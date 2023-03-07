import sys
import time
import copy
import rospy
import numpy as np
import moveit_commander as mc
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.srv import GetLinkState, SetLinkProperties

def get_link_pose(name: str) -> Pose:
    """
    Retrieve an object pose from the gazebo world.

    Parameters
    ----------
    name : string
        name of the object in the gazebo world

    Returns
    -------
    `Pose` : current pose of the object in the gazebo world
    """
    rospy.wait_for_service('/gazebo/set_link_properties')
    link_state_client = rospy.ServiceProxy( '/gazebo/set_link_properties', SetLinkProperties)
    link_state_client.call(link_name=name, gravity_mode=False)

get_link_pose(name='object')