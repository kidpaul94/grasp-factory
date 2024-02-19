import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetWorldProperties, SpawnModel, DeleteModel, SetLinkProperties, GetModelState

class EnvManager():
    def __init__(self) -> None:
        self.permanet_objects = self.get_gazebo_objects()
        self.added_objects = []

    def get_gazebo_objects(self) -> list:
        """
        Save a list of objects in the gazebo world.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        obj_list : 1xN : obj : `list`
            object list composed with Models
        """
        rospy.wait_for_service('/gazebo/get_world_properties')
        world_state_client = rospy.ServiceProxy( '/gazebo/get_world_properties', GetWorldProperties)
        obj_names, obj_list = world_state_client.call().model_names, []
        for name in obj_names:
            obj_list.append(name)

        return obj_list

    def sync_with_gazebo(self) -> None:
        """
        Sync EnvManager objects with the list of objects in
        the gazebo world after deletion.

        Parameters
        ----------
        None
            
        Returns
        -------
        None
        """
        check_objects = self.get_gazebo_objects()
        added_org = set(temp[0] for temp in self.added_objects)
        added_left, added_perm = [], []
        for obj in check_objects:
            if obj in added_org:
                added_left.append(obj)
            else:
                added_perm.append(obj)

        self.added_objects = added_left
        self.permanet_objects = added_perm
        
    def spawn_object(self, name: str, pose: Pose, sdf_name: str) -> None:
        """
        Spawn an object in the gazebo world.

        Parameters
        ----------
        name : string
            name of the object in the gazebo world
        pose : obj : `Pose`
            pose of the object when spawning
        sdf_name : string
            sdf file of the object we spawn in the gazebo world

        Returns
        -------
        None
        """
        spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_client(model_name=name,
        model_xml=open(f'../../models/{sdf_name}/model.sdf', 'r').read(),
        robot_namespace='/foo', initial_pose=pose, reference_frame='world')
        self.added_objects.append([name, sdf_name])

    @staticmethod
    def delete_object(name: str) -> None:
        """
        Delete an object in the gazebo world.

        Parameters
        ----------
        name : string
            name of the object in the gazebo world

        Returns
        -------
        None
        """
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model_client.call(model_name=name)

    @staticmethod
    def set_link_prop(name: str) -> None:
        """
        Set object link properties int the gazebo world.

        Parameters
        ----------
        name : string
            name of the object link in the gazebo world

        Returns
        -------
        None
        """
        rospy.wait_for_service('/gazebo/set_link_properties')
        link_state_client = rospy.ServiceProxy( '/gazebo/set_link_properties', SetLinkProperties)
        link_state_client.call(link_name=name, gravity_mode=True)

    @staticmethod
    def get_object_pose(name: str) -> Pose:
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
        rospy.wait_for_service('/gazebo/get_model_state')
        model_state_client = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
        state = model_state_client.call(model_name=name, relative_entity_name='map')

        return state.pose
