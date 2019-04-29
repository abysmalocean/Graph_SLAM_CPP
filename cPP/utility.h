#ifndef _UTILITY_LIB
#define _UTILITY_LIB

#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <math.h>



struct Edge
{
    Edge(std::ifstream& fileLine){
        fileLine >> idFrom ; 
        fileLine >> idTo ; 
        fileLine >> meanX ;  
        fileLine >> meanY ;  
        fileLine >> meanTheta;   
        fileLine >> infXX; 
        fileLine >> infXY;  
        fileLine >> infYY;  
        fileLine >> infXT;  
        fileLine >> infYT;  
        fileLine >> infTT; 
        genMatrix(); 

    }
    int idFrom;
    int idTo;
    double meanX; 
    double meanY; 
    double meanTheta;  
    double infXX;
    double infXY; 
    double infYY; 
    double infXT; 
    double infYT; 
    double infTT;
    Eigen::Vector3d mean; 
    Eigen::Matrix3d infm; 

    void genMatrix()
    {
        mean << meanX,  meanY, meanTheta; 
        infm << infXX, infXY, infXT, infXY, infYY, infYT, infXT,
                infYT, infTT; 
    }

}; 

struct Vertex
{
    Vertex(int id, double poseX, double poseY, double poseThe): 
        id(id), poseX(poseX), poseY(poseY), poseThe(poseThe)
    {
        mean << poseX, poseY, poseThe; 
    }
    int id; 
    double poseX; 
    double poseY; 
    double poseThe;
    Eigen::Vector3d mean; 
    std::vector<Edge*> in; 
    std::vector<Edge*> out; 
}; 


Eigen::Matrix3d v2t(Eigen::Vector3d& Vec)
{
    /*
    Matrix3f m;
    m << 1, 2, 3,
     4, 5, 6,
     7, 8, 9;
    */
    double c = std::cos(Vec(2)); 
    double s = std::sin(Vec(2)); 
    Eigen::Matrix3d Trans;
    Trans << c , -s, Vec(0), s, c, Vec(1), 0, 0, 1; 
    return Trans; 
}

Eigen::Vector3d t2v(Eigen::Matrix3d& Trans)
{
    double ang =  std::atan2(Trans(0,0), Trans(0,0)); 
    Eigen::Vector3d Vec; 
    Vec << Trans(0,2), Trans(1,2), ang; 
    return Vec;
} 



#endif