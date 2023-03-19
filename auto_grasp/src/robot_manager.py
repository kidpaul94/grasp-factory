import sys
import rospy
import moveit_commander as mc
from gazebo_msgs.srv import GetJointProperties

class Move_Robot():
    def __init__(self) -> None:
        mc.roscpp_initialize(sys.argv)
        self.gripper = mc.MoveGroupCommander("gripper_fingers")

    def gripper_control(self, command: bool = True) -> None:
        """
        Open or close gripper. 

        Parameters
        ----------
        command: bool
            close (True) or open (False) the robot gripper

        Returns
        -------
        None
        """
        if command:
            self.gripper.go([0.0065, -0.0065], wait=True)
        else:
            self.gripper.go([-0.021, 0.021], wait=True)
        self.gripper.stop()

    @staticmethod
    def is_grasped(joint_names: list = ['Slider01', 'Slider02'], close_pos: list = [0.0065, -0.0065],
                   threshold: float = 0.0005):
        """
        Check if an object is successfully grasped.

        Parameters
        ----------
        joint_names : 1xN : obj : `list`
            list of gripper joint names
        close_pos : 1xN : obj : `list`
            list of closed gripper joint positions
        threshold : float
            threshold value to determin whether the object is 
            within two fingers

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
