# -*- coding: cp936 -*-
import math
import numpy as np

def t2v(A):
    #A为3*3矩阵
    v=np.mat(np.zeros([3,1]))
    v[0,0]=A[0,2]
    v[1,0]=A[1,2]
    v[2,0]=math.atan2(A[1,0],A[0,0])
    return v

def v2t(v):
    #v为3*1矩阵
    if v.shape !=(3,1):
        print 'v is not 3*1 maxtir'
        return 0
    c=math.cos(v[2,0])
    s=math.sin(v[2,0])
    A=np.mat([[c,-s,v[0,0]],[s,c,v[1,0]],[0,0,1]])
    return A

if __name__=='__main__':
    v=np.mat([1,2]).T
    A=v2t(v)
    print A
    #d=np.concatenate((c,a.T),axis=1)#axis=1按列方向上合并；axis=0按行方向上合并
    #v1=t2v(A)
    #print v1
