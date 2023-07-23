import sys
import rospy
import moveit_commander as mc
from gazebo_msgs.srv import GetJointProperties

class Move_Robot():
    def __init__(self) -> None:
        mc.roscpp_initialize(sys.argv)
        self.gripper = mc.MoveGroupCommander("gripper_fingers")

    def gripper_control(self, width: float, command: bool = True) -> None:
        """
        Open or close gripper. 

        Parameters
        ----------
        width: float
            targed closing width of the robot gripper
        command: bool
            close (True) or open (False) the robot gripper

        Returns
        -------
        target_config: 1x2 : obj : `list`
            target gripper configuration
        """
        if command:
            target_config = [0.009 - 0.001 * width, -0.009 + 0.001 * width]
        else:
            target_config = [-width, width]
        self.gripper.go(target_config, wait=True) # 0.0065
        self.gripper.stop()

        return target_config

    @staticmethod
    def is_grasped_F(joint_names: list = ['Slider01', 'Slider02'], close_pos: list = [0.0065, -0.0065],
                   threshold: float = 0.0005):
        """
        Check whether the object is grasped or not by comparing commanded 
        and actual gripper configurations (This is more useful for full-stroke grasping). 

        Parameters
        ----------
        joint_names : 1xN : obj : `list`
            list of gripper joint names
        close_pos : 1xN : obj : `list`
            list of closed gripper joint positions
        threshold : float
            threshold value to determin whether the object is within two fingers

        Returns
        -------
        bool : whether succeed in grasping    
        """
        rospy.wait_for_service('/gazebo/get_joint_properties')
        link_state_client = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        for idx, name in enumerate(joint_names):
            state = link_state_client.call(joint_name=name)
            if abs(close_pos[idx] - state.position[0]) < threshold:   
                return False

        return True

    @staticmethod
    def is_grasped_Z(obj_positon_z: float, threshold: float = 0.5):
        """
        Check whether the object is grasped or not by checking the z position
        of the object.

        Parameters
        ----------
        obj_positon_z : float
            z position of the object after grasping
        threshold : float
            threshold value to determin whether the object is fallen or not

        Returns
        -------
        bool : whether succeed in grasping    
        """
        if obj_positon_z < threshold:   
            return False

        return True
    
