import os
import time
import rospy
import argparse
import numpy as np
from tqdm import tqdm

from utils import Conversion
from env_manager import EnvManager
from robot_manager import Move_Robot

def parse_args(argv=None) -> None:
    parser = argparse.ArgumentParser(description='auto_grasp')
    parser.add_argument('--name', default='object_1', type=str,
                        help='name of the object in the gazebo world.')
    parser.add_argument('--sixd', default=[0., 0., 0.65, 0, 0, 0.], type=list, 
                        help='list of xyz and three euler angles.')
    parser.add_argument('--sdf_names', default=['obj_05'], type=str, nargs='+',
                        help='sdf files of objects we sequentially spawn in the gazebo world.')
    parser.add_argument('--grasp_dicts', default='../../models/dict', type=str,
                        help='root_dir to a .txt file of cpps.')
    parser.add_argument('--iter_sample', default=1, type=int,
                        help='number of iteration per grasp sample.')
    parser.add_argument('--save_probs', default=True, type=bool,
                        help='whether to save the simulation results.')

    global args
    args = parser.parse_args(argv)

class Auto_grasp():
    def __init__(self, args):
        self.mr = Move_Robot()
        self.EM, self.conv = EnvManager(), Conversion()
        self.pose = self.conv.list2pose(sixd=args.sixd)
        self.name, self.sdf_names = args.name, args.sdf_names
        self.grasp_dicts, self.iter_sample = args.grasp_dicts, args.iter_sample

    def execute(self, center: np.ndarray, direction: np.ndarray, sdf_name: str, 
                repeat: int = 100) -> float:
        """
        Execute a given grasp configuration # times to obtain its probability of success

        Parameters
        ----------
        center : 3xN : obj : `np.ndarray`
            array of potential gripper centers w.r.t the world frame
        direction : 3xN : obj : `np.ndarray`
            array of potential gripper directions w.r.t the world frame
        sdf_name : string
            sdf file of objects we spawn in the gazebo world
        repreat : int
            number of iteration to execute the grasp configuration

        Returns
        -------
        float : probability of successing the grasp
        """
        attempt, total = 0, repeat
        for _ in range(repeat):
            self.mr.gripper_control(command=False)
            time.sleep(1)
            print('Object is inbetween the gripper fingers...')
            self.mr.gripper_control()
            time.sleep(1)

            if self.mr.is_grasped():
                attempt = attempt + 1
            print(f'{attempt} out of {total} succeeded!!!')

            self.EM.delete_object(name=self.name)
            time.sleep(1)
            self.EM.spawn_object(name=self.name, pose=self.pose, sdf_name=sdf_name)
            time.sleep(1)
            self.EM.sync_with_gazebo()
            time.sleep(1)

        return attempt / total if total >= 1 else -1.0

    def initialize(self) -> None:
        """
        Initialize auto grasp process by spawning a target object and calculate 
        grasp configurations w.r.t the world frame.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        directory = f'result'
        if not os.path.exists(directory):
            print(f'Generate {directory} folder...')
            os.mkdir(directory)

        for obj in self.sdf_names:
            success_prob, entire_list = [], []
            self.EM.spawn_object(name=self.name, pose=self.pose, sdf_name=obj)
            time.sleep(1)
            # self.EM.set_link_prop(name='object') # must be a link name in model.sdf
            self.EM.sync_with_gazebo()
            time.sleep(1)
            self.mr.gripper_control(command=False)

            path_dict = f'{self.grasp_dicts}/{obj}.txt'
            centers, directions = self.EM.added_objects[0].grasp_gen(path=path_dict)
            iteration = len(directions)

            for idx in tqdm(range(iteration)):
                res = self.execute(centers[:,idx], directions[:,idx], obj, self.iter_sample)
                if res >= 0:
                    success_prob.append(res)
                    print(f'Object: {obj}')
                    print(f'Probabilities: {success_prob}')
                    print(f'Mean: {np.mean(success_prob)}, STD: {np.std(success_prob)}')
                entire_list.append(res)

            self.EM.delete_object(name=self.name)
            time.sleep(1)

            if args.save_probs:
                with open(f'{directory}/{obj}.txt', 'w') as output:
                    print(f'Generate {obj} grasp dictionaries...')
                    output.write(repr(entire_list))
                    output.close()

        print('Finished the simulation!!!')
    
if __name__ == "__main__":
    parse_args()
    rospy.init_node('auto_grasp', anonymous=True)
    Auto_grasp(args).initialize()
