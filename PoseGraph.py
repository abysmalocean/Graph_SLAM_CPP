# -*- coding: cp936 -*-
import math
import numpy as np
from PoseNode import PoseNode
from PoseEdge import PoseEdge
import convert
import pylab
import numpy as np
import math
import time
import scipy.sparse as ss
import scipy.sparse.linalg as ssl

def id2ind(Id):
    ind=[(3*Id),(3*(Id+1))]
    return ind

class PoseGraph(object):
    node=[]  #Pose nodes in graph
    edge=[]  #Edge in graph
    #H     #Information matrix
    #b     #Information vector
    #n_node #number of nodes in graph
    #n_edge #number of edges in graph
    #pose   #Poses of all nodes
    def __init__(self):
        node=[]
        edge=[]

    def readGraph(self,vfile,efile):
        
        with open(vfile,'r') as f:
            for line in f.readlines():
                a=line.strip().split(' ')
                vi=[float(x) for x in a[1:]]
                Id=vi[0];
                pose=vi[1:4]
                self.node.append(PoseNode(Id,pose))

        print('Vertices loaded from: '+vfile)
        self.n_node=len(self.node)

        with open(efile,'r') as f:
            for line in f.readlines():
                a=line.strip().split(' ')
                ei=[float(x) for x in a[1:]]
                id_from=ei[0]
                
                id_to=ei[1]
                
                mean=ei[2:5]
                
                infm=np.mat(np.zeros([3,3]))
                infm[0,0]=ei[5]
                infm[1,0]=ei[6]
                infm[0,1]=ei[6]
                infm[1,1]=ei[7]
                infm[2,2]=ei[8]
                infm[0,2]=ei[9]
                infm[2,0]=ei[9]
                infm[2,1]=ei[10]
                infm[1,2]=ei[10]
                
                self.edge.append(PoseEdge(id_from,id_to,mean,infm))

        print ('Edges loaded from: '+efile)
        self.n_edge=len(self.edge)

    def plot(self):
        X=[each_node.pose[0,0] for each_node in self.node]
        Y=[each_node.pose[1,0] for each_node in self.node]
        pylab.plot(X,Y)

    def optimize(self,n_iter=1,vis=False):
        
        for i in range(n_iter):
            print 'Pose Graph Optimization, Iteration %d.'%(i+1)
            self.iterate()
            print 'Iteration %d done.'%(i+1)

            if vis:
                pylab.clf()
                #pylab.ion()
                self.plot()
                pylab.title('Iteration %d'%(i+1))
                #pylab.draw()
                #pylab.ioff()
                #time.sleep(3)
                pylab.show()
        #pylab.show()

    def iterate(self):
        print ('Allocating Workspace.')

        self.H=np.mat(np.zeros([self.n_node*3,self.n_node*3]))
        self.b=np.mat(np.zeros([self.n_node*3,1]))

        print('Linearizing.')
        self.linearize()

        print('Solving.')
        self.solve()

    def linearize(self):
        for ei in self.edge:
            i_node=ei.id_from
            j_node=ei.id_to
            T_z=convert.v2t(ei.mean)
            omega=ei.infm

            v_i=self.node[int(i_node)].pose
            v_j=self.node[int(j_node)].pose
            i_ind=id2ind(int(i_node))
            j_ind=id2ind(int(j_node))

            T_i=convert.v2t(v_i)
            T_j=convert.v2t(v_j)            
            R_i=T_i[0:2,0:2]
            R_z=T_z[0:2,0:2]

            si=math.sin(v_i[2,0])
            ci=math.cos(v_i[2,0])
            dR_i=np.mat([[-si,ci],[-ci,-si]]).T
            dt_ij=v_j[0:2,0]-v_i[0:2,0]

            A=np.concatenate((np.concatenate((-R_z.T*R_i.T,R_z.T*dR_i.T*dt_ij),axis=1),np.mat([0,0,-1])),axis=0)
            B=np.concatenate((np.concatenate((R_z.T*R_i.T,np.mat([0,0]).T),axis=1),np.mat([0,0,1])),axis=0)

            e=convert.t2v(T_z.I*T_i.I*T_j)

            H_ii=A.T*omega*A
            H_ij=A.T*omega*B
            H_jj=B.T*omega*B
            b_i=-A.T*omega*e
            b_j=-B.T*omega*e

            self.H[i_ind[0]:i_ind[1],i_ind[0]:i_ind[1]]=self.H[i_ind[0]:i_ind[1],i_ind[0]:i_ind[1]]+H_ii
            self.H[i_ind[0]:i_ind[1],j_ind[0]:j_ind[1]]=self.H[i_ind[0]:i_ind[1],j_ind[0]:j_ind[1]]+H_ij
            self.H[j_ind[0]:j_ind[1],i_ind[0]:i_ind[1]]=self.H[j_ind[0]:j_ind[1],i_ind[0]:i_ind[1]]+H_ij.T
            self.H[j_ind[0]:j_ind[1],j_ind[0]:j_ind[1]]=self.H[j_ind[0]:j_ind[1],j_ind[0]:j_ind[1]]+H_jj
            self.b[i_ind[0]:i_ind[1]]=self.b[i_ind[0]:i_ind[1]]+b_i
            self.b[j_ind[0]:j_ind[1]]=self.b[j_ind[0]:j_ind[1]]+b_j

    def solve(self):
        print 'Pose: %d, Edge: %d'%(self.n_node,self.n_edge)

        self.H[0:3,0:3]=self.H[0:3,0:3]+np.eye(3)
        H=self.H.copy()
        #把矩阵H变为一个稀疏矩阵，储存形式为CSC格式
        H_sparse=ss.csc_matrix(H)
        #计算稀疏矩阵A的LU分解，A应采用CSR或CSC格式，返回SuperLU对象
        invhs=ssl.splu(H_sparse)
        #SuperLU对象的solve方法，用于求解具有一个或者多个右侧的线性方程组
        '''solve（rhs [，trans]）
        rhs:ndarray(矩阵)，shape为 n 或者（n，k），方程组的右手边
        trans：{'N'，'T'，'H'}，可选
        要解决的系统类型：
        'N'：A * x == rhs（默认）
         'T'：A ^ T * x == rhs
         'H'：A ^ H * x == rhs
        '''
        dx=invhs.solve(self.b)
        
        dpose=dx.reshape([3,self.n_node],order='F')
        #print dpose

        for i_node in range(self.n_node):
            self.node[i_node].pose=self.node[i_node].pose+dpose[:,i_node]
        
        
