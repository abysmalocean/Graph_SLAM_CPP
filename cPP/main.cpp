#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include <Eigen/SparseQR>
// for sparse Matrix
#include <Eigen/SparseCore>

// Eigen tutorial 
// https://eigen.tuxfamily.org/dox/group__TutorialSparse.html

#include "utility.h"

// read the graph

using std::cout; 
using std::endl; 

// declares a column-major sparse matrix type of double
typedef Eigen::SparseMatrix<double> SpMat; 


class PoseGraph
{

public: 
    PoseGraph(std::string vfile, std::string efile) : 
    vfile(vfile), efile(efile)
    {}

    bool readGraph()
    {
        std::ifstream VFile(vfile);
        if(!VFile.is_open())
        {
            cout << "Vfile is not opne, for some reason" << endl; 
        }
        std::string str;
        std::string VertexName;  
        int tmp_id; 
        double tmp_poseX; 
        double tmp_poseY; 
        double tmp_poseThe;
        int count = 0;  
        while(getline(VFile, str))
        {
            VFile >> VertexName;
            if (VertexName == "VERTEX2")
            { 
                VFile >> tmp_id; 
                VFile >> tmp_poseX; 
                VFile >> tmp_poseY; 
                VFile >> tmp_poseThe; 
                //cout << tmp_id << " " << tmp_poseX << " " << tmp_poseY << endl; 
                V.push_back(new Vertex(tmp_id, tmp_poseX, tmp_poseY, tmp_poseThe)); 
            }
            VertexName = ""; 
        }
        cout <<"Load Vertex : " << V.size() << endl; 
        VFile.close(); 

        std::ifstream EFile(efile); 
        if (!EFile.is_open())
        {
            cout << "Efile is not opne, for some reason" << endl; 
        }
        count = 0; 
        while(getline(EFile, str))
        {
            ++count; 
            EFile >> VertexName; 
            if (VertexName == "EDGE2")
            {
                E.push_back(new Edge(EFile));
                // cout << E.back()->idFrom << endl; 
                V[E.back()->idFrom]->out.push_back(E.back()); 
                V[E.back()->idTo]->in.push_back(E.back()); 
            }
            
        }
        cout <<"Load Edge : " << count << endl; 
    }

    void optimize(int n_iter=1, bool vis=false)
    {
        for (int i = 0; i < n_iter; ++i)
        {
            cout << "Pose Graph Optimization, Iteration " << i << endl; 
            iterate(); 
            cout << "Iteration " << i << endl;  
        }
    }

private: 
    void iterate()
    {
        int size = V.size() * 3; 
        SpMat H(size,size);
        Eigen::VectorXd b(size); 
        linearize(H, b); 

    }
    void linearize(SpMat &H, Eigen::VectorXd& b)
    {
        Eigen::Matrix3d Trans; 
            t2v(Trans); 
        for (int i = 0; i < E.size(); ++i)
        {
            // TODO: Build the Static Graph
            
        }

    }

private: 
    std::string vfile; 
    std::string efile;
    std::vector<Vertex*> V; 
    std::vector<Edge*> E; 

}; 


int main(int argc, char** argv)
{
    cout << "Begin the graph_slam algorithm" << endl; 
    std::string vfile= "../data/killian-v.dat";
    std::string efile="../data/killian-e.dat"; 

    PoseGraph poseGraph(vfile, efile); 
    poseGraph.readGraph();
    poseGraph.optimize(5); 

    return 0; 


}