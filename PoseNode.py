# -*- coding: cp936 -*-
import math
import numpy as np
import convert

class PoseNode(object):
    #�β�poseΪ����Ϊ3��list
    
    def __init__(self,Id,pose):
        self.Id=Id
        self.pose=np.mat(pose).T #obj.pose = pose(:);ת��Ϊ3*1�ľ�����������

        self.x=self.pose[0,0]
        self.y=self.pose[1,0]
        self.yaw=self.pose[2,0]
        self.rt=convert.v2t(self.pose)
    
