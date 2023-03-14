import os
import time
import rospy
import argparse
import numpy as np
from tqdm import tqdm

from env_manager import EnvManager
from robot_manager import Move_Robot
from utils import Conversion, grasp_gen

def parse_args(argv=None) -> None:
    parser = argparse.ArgumentParser(description='auto_grasp')
    parser.add_argument('--name', default='object_1', type=str,
                        help='name of the object in the gazebo world.')
    parser.add_argument('--sixd', default=[0., 0., 0.85, 0., 0., 0.], type=list, 
                        help='list of xyz and three euler angles.')
    parser.add_argument('--sdf_names', default=['obj_05'], type=str, nargs='+',
                        help='sdf files of objects we sequentially spawn in the gazebo world.')
    parser.add_argument('--grasp_dicts', default='../../models/dict', type=str,
                        help='root_dir to a .txt file of cpps.')
    parser.add_argument('--save_probs', default=True, type=bool,
                        help='whether to save the simulation results.')

    global args
    args = parser.parse_args(argv)

class Auto_grasp():
    def __init__(self, args):
        self.mr = Move_Robot()
        self.EM, self.conv = EnvManager(), Conversion()
        self.init_T = self.conv.list2T(sixd=args.sixd)
        self.name, self.sdf_names = args.name, args.sdf_names
        self.grasp_dicts = args.grasp_dicts

    def execute(self, center: np.ndarray, direction: np.ndarray, sdf_name: str, 
        aprvs: list) -> float:
        """
        Execute a given grasp configuration # times to obtain its probability of success

        Parameters
        ----------
        center : 3x1 : obj : `np.ndarray`
            gripper center w.r.t the world frame (in mm)
        direction : 3x1 : obj : `np.ndarray`
            gripper direction w.r.t the world frame
        sdf_name : string
            sdf file of objects we spawn in the gazebo world
        aprvs : 1xM : obj : `list`
            number of iteration to execute the grasp configuration

        Returns
        -------
        float : probability of successing the grasp
        """
        attempt, total = 0, len(aprvs)
        for aprv in aprvs:
            self.mr.gripper_control(command=False)
            time.sleep(0.5)
            # T = grasp config w.r.t an object frame
            unit_direction = direction / np.linalg.norm(direction)
            T_inv = self.conv.cpp2T(center=center, direction=unit_direction, aprv=aprv)
            spawn_pose = self.conv.T2pose(self.init_T @ T_inv) 

            self.EM.spawn_object(name=self.name, pose=spawn_pose, sdf_name=sdf_name)
            time.sleep(0.5)
            self.EM.sync_with_gazebo()
            time.sleep(0.5)
            self.mr.gripper_control()
            time.sleep(1.0)
            self.EM.set_link_prop(name='object') # must be a link name in model.sdf
            time.sleep(1.0)

            if self.mr.is_grasped():
                attempt = attempt + 1
            print(f'{attempt} out of {total} succeeded!!!')

            self.EM.delete_object(name=self.name)
            time.sleep(0.5)

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
        directory = 'result'
        if not os.path.exists(directory):
            print(f'Generate {directory} folder...')
            os.mkdir(directory)

        for obj in self.sdf_names:
            entire_list = []
            patch_dict = f'{self.grasp_dicts}/{obj}'
            centers, directions, aprvs = grasp_gen(path=patch_dict)

            for idx in tqdm(range(directions.shape[1])):
                prob = self.execute(centers[:,idx], directions[:,idx], obj, aprvs[idx])
                if prob >= 0:
                    print(f'Object: {obj}')
                    print(f'Probabilities: {prob}')
                entire_list.append(prob)

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
