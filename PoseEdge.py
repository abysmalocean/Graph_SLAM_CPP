# -*- coding: cp936 -*-
import numpy as np

class PoseEdge(object):
    #�β�meanΪ����Ϊ3��list��infmΪ3*3����
    def __init__(self,id_from,id_to,mean,infm):
        
        self.id_from=id_from
        self.id_to=id_to
        self.mean=np.mat(mean).T #ת��Ϊ3*1�ľ�����pose��ͬ
        self.infm=infm
