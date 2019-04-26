# -*- coding: cp936 -*-
import math
import numpy as np
import convert

class PoseNode(object):
    #形参pose为长度为3的list
    
    def __init__(self,Id,pose):
        self.Id=Id
        self.pose=np.mat(pose).T #obj.pose = pose(:);转换为3*1的矩阵（列向量）

        self.x=self.pose[0,0]
        self.y=self.pose[1,0]
        self.yaw=self.pose[2,0]
        self.rt=convert.v2t(self.pose)
    
