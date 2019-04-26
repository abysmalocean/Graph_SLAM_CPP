from PoseGraph import PoseGraph

vfile='data/killian-v.dat'
efile='data/killian-e.dat'

pg=PoseGraph()
pg.readGraph(vfile,efile)

pg.optimize(5,True)
