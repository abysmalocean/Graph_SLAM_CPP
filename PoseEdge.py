# -*- coding: cp936 -*-
import numpy as np

class PoseEdge(object):
    #形参mean为长度为3的list，infm为3*3矩阵
    def __init__(self,id_from,id_to,mean,infm):
        
        self.id_from=id_from
        self.id_to=id_to
        self.mean=np.mat(mean).T #转换为3*1的矩阵，与pose相同
        self.infm=infm
