#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

//#include <Eigen/SparseQR>
#include<Eigen/SparseLU>
//#include<Eigen/SparseCholesky>
// for sparse Matrix
#include <Eigen/SparseCore>
#include <Eigen/Dense>

#include "utility.h"

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
                V[E.back()->idFrom]->out.push_back(E.back()); 
                V[E.back()->idTo]->in.push_back(E.back()); 
            }
            
        }
        cout <<"Load Edge : " << count << endl; 
    }

    void optimize(int n_iter=1, bool vis=false)
    {
        double error = 1.79769e+308; 
        for (int i = 0; i < n_iter; ++i)
        {
            cout << "Pose Graph Optimization, Iteration " << i << endl; 
            double currentError = iterate(i); 
            if (std::abs(currentError - error) < 0.1)
            {
                cout << "Solution is in the minimum" << endl; 
                return; 
            }
            error = currentError; 
        }
    }

private: 
    double iterate(int iter)
    {
        int size = V.size() * 3; 
        SpMat H(size,size);
        Eigen::VectorXd b(size); 
        b << Eigen::VectorXd::Zero(size);

        double error =  linearize(H, b); 
        solve(H,b); 
        return error; 
    }

    double linearize(SpMat &H, Eigen::VectorXd& b)
    {
        double totalError = 0.0; 
        for (int i = 0; i < E.size(); ++i)
        {
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
            dt_ij << v_j->poseX - v_i->poseX, v_j->poseY - v_i->poseY; 
            
            Eigen::Matrix3d A_ij; 

            A_ij.block(0,0,2,2) = -R_ij.transpose() * R_i.transpose(); 
            auto tmp = R_ij.transpose() * dR_i * dt_ij; 
            A_ij(0,2) = tmp(0); 
            A_ij(1,2) = tmp(1);

            A_ij(2,0) = 0.0; 
            A_ij(2,1) = 0.0; 
            A_ij(2,2) = -1.0; 

            //cal B_IJ
            Eigen::Matrix3d B_ij;
            B_ij << R_ij.transpose() * R_i.transpose() , 
                    Eigen::MatrixXd::Zero(2,1), Eigen::MatrixXd::Zero(1,2), 1.0 ;

            Eigen::Matrix3d Trans = (T_ij.inverse() * (T_i.inverse() * T_j));
            Eigen::Vector3d e = t2v(Trans); 

            totalError +=  e.transpose() * omega * e;

            Eigen::Matrix3d H_ii = A_ij.transpose() * omega * A_ij; 
            Eigen::Matrix3d H_ij = A_ij.transpose() * omega * B_ij; 
            Eigen::Matrix3d H_jj = B_ij.transpose() * omega * B_ij; 

            Eigen::Vector3d b_i = -A_ij.transpose() * omega * e; 
            Eigen::Vector3d b_j = -B_ij.transpose() * omega * e; 

            // because the sparce matrix in the block operatation is static. 
            updateH(E[i]->idFrom, E[i]->idFrom, H, H_ii); 
            updateH(E[i]->idFrom, E[i]->idTo,   H, H_ij); 
            updateH(E[i]->idTo,   E[i]->idFrom, H, H_ij.transpose()); 
            updateH(E[i]->idTo,   E[i]->idTo,   H, H_jj); 

            b.segment(3 * E[i]->idFrom, 3) += b_i; 
            b.segment(3 * E[i]->idTo  , 3) += b_j;
    
        }

        cout << "Total error " << totalError << endl; 
        return totalError; 
    }

    void solve(SpMat &H, Eigen::VectorXd& b)
    {
        //cout << b << endl; 
        H.coeffRef(0 , 0) += 1.0; 
        H.coeffRef(1 , 1) += 1.0;
        H.coeffRef(2 , 2) += 1.0;

        Eigen::SparseLU<SpMat> solver; 
        solver.compute(H); 
        if (solver.info() != Eigen::Success)
        {
            cout << "decomposition Sparce matrix have some problem " << endl; 
            return; 
        }
        Eigen::VectorXd dx; 
        dx = solver.solve(b);

        if (solver.info() != Eigen::Success)
        {
            cout << "Sparce matrix have some problem " << endl; 
            return; 
        }

        for (int i = 0; i < V.size(); ++i)
        {
            
            V[i]->poseX   += dx[i*3 ]; 
            V[i]->poseY   += dx[i*3 + 1]; 
            V[i]->poseThe += dx[i*3 + 2];

            V[i]->mean(0) += dx[i*3 ];
            V[i]->mean(1) += dx[i*3 + 1];
            V[i]->mean(2) += dx[i*3 + 2];
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
    poseGraph.optimize(10); 

    return 0; 


}