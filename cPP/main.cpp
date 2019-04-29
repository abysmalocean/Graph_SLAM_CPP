#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include <Eigen/SparseQR>
// for sparse Matrix
#include <Eigen/SparseCore>
#include <Eigen/Dense>


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
        Eigen::MatrixXd H_dense(size,size); 
        linearize(H, b); 

    }

    void linearize(SpMat &H, Eigen::VectorXd& b)
    {
        // i < E.size(); 2 for testing
        for (int i = 0; i < 2; ++i)
        {
            // TODO: Build the Static Graph
            Eigen::Matrix3d T_ij = v2t(E[i]->mean); 
            Eigen::Matrix3d omega = E[i]->infm;
            Vertex* v_i = V[E[i]->idFrom];
            Vertex* v_j = V[E[i]->idTo]; 

            Eigen::Matrix3d T_i = v2t(v_i->mean);
            Eigen::Matrix3d T_j = v2t(v_j->mean); 

            Eigen::Matrix2d R_ij = T_ij.block(0,0,2,2); 
            Eigen::Matrix2d R_i  =  T_i.block(0,0,2,2); 

            // cal A_IJ
            double si = std::sin(v_i->poseThe); 
            double ci = std::cos(v_i->poseThe); 
            Eigen::Matrix2d dR_i; 
            dR_i << -si, ci , -ci, -si; 
            Eigen::Vector2d dt_ij; 
            dt_ij << v_j->poseX - v_i->poseX, v_j->poseY - v_j->poseY; 
            Eigen::Matrix3d A_ij; 
            A_ij.block(0,0,2,2) = -R_ij.transpose() * R_i.transpose(); 
            auto tmp = R_ij * dR_i*dt_ij; 
            
            A_ij(0,2) = tmp(0); 
            A_ij(1,2) = tmp(1);

            A_ij(2,0) = 0.0; 
            A_ij(2,1) = 0.0; 
            A_ij(2,2) = -1.0; 


            //cal B_IJ
            Eigen::Matrix3d B_ij;
            B_ij << R_ij.transpose() * R_i.transpose() , 
                    Eigen::MatrixXd::Zero(2,1), Eigen::MatrixXd::Zero(1,2), 1.0 ;

            //cout << "A : \n" <<A_ij << endl; 
            //cout << "B : \n" << B_ij << endl; 
            // cout << T_i.inverse() << endl; 
            Eigen::Matrix3d Trans = (T_ij.inverse() * (T_i.inverse() * T_j));
            //cout << something << endl; 
            Eigen::Vector3d e = t2v(Trans); 

            Eigen::Matrix3d H_ii = A_ij.transpose() * omega * A_ij; 
            Eigen::Matrix3d H_ij = A_ij.transpose() * omega * B_ij; 
            Eigen::Matrix3d H_jj = B_ij.transpose() * omega * B_ij; 

            Eigen::Vector3d b_i = A_ij.transpose() * omega * e; 
            Eigen::Vector3d b_j = B_ij.transpose() * omega * e; 
            cout << A_ij.size() <<endl; 
            // TODO: some bugs here, the sparse matrix have some problem
            H.block(E[i]->idFrom, E[i]->idFrom, 3, 3)  =  H_ii.block(0,0,3,3); 
            //H.block(E[i]->idFrom, E[i]->idTo, 3, 3)    =  H_ij; 
            //H.block(E[i]->idTo, E[i]->idFrom, 3, 3)    =  H_ij.transpose();
            //H.block(E[i]->idTo, E[i]->idTo, 3, 3)      =  H_jj;

            b.segment(E[i]->idFrom, 3) += b_i; 
            b.segment(E[i]->idTo  , 3) += b_j; 

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