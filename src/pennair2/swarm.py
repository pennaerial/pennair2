# Copyright (C) 2018  Penn Aerial Robotics
# Fill copyright notice at github.com/pennaerial/pennair2/NOTICE

from core import UAV
import copy

class Swarm():
    def __init__(self):
        self.members = []
        self.formation = None
        self.margin = 5 #The offset to use when calculating group pose

    def add_member(self, m):
        self.members.append(m)

    def remove_member(self, m):
        self.member.remove(m)

    def set_position(self, value, frame_id=None, heading=None):
        """
        
        :param value: The desired position setpoint, only yaw component of orientation is used. Can be of type
            PoseStamped, Pose, Point, or an indexable object with 3 integer elements (list, tuple, numpy array etc.)
        :type value: PoseStamped | Pose | Point | list[int,int,int] | (int,int,int)
        :param frame_id: The name of the frame to use for the message.
        :type frame_id: str
        :param heading: Your desired heading.
        :type heading: int
        """
        origin_msg = UAV.generate_pose_stamped(value, frame_id, heading)
        poses = self.get_swarm_poses(origin_msg)
        for uav, pose in zip(self.members, poses):
            uav.set_position(pose)

    def get_swarm_poses(self, origin):
        #TODO: These calculations should be based on formation, for now we'll just always add 5 to x
        pl = []
        offset = origin.pose.position.x
        for x in self.members:
            msg = copy.deepcopy(origin)
            msg.pose.position.x = offset
            pl.append(msg)
            offset += self.margin

        return pl
