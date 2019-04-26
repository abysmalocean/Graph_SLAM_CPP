#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>


// read the graph

using std::cout; 
using std::endl; 

struct Vertex
{
    int id; 
    double poseX; 
    double poseY; 
    double poseThe;
}; 

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
        int tmp_id; 
        double tmp_poseX; 
        double tmp_poseY; 
        double tmp_poseThe; 
        while(getline(VFile, str))
        {
            VFile >> str; 
            VFile >> tmp_id; 
            VFile >> tmp_poseX; 
            VFile >> tmp_poseY; 
            VFile >> tmp_poseThe; 
            cout << str << "   " <<tmp_id << " " << tmp_poseX << " " << tmp_poseY << endl; 
        }

        std::ifstream EFile(efile); 

    }
private: 
    std::string vfile; 
    std::string efile; 

}; 


int main(int argc, char** argv)
{
    cout << "Begin the graph_slam algorithm" << endl; 
    std::string vfile= "data/killian-v.dat";
    std::string efile="data/killian-e.dat"; 

    PoseGraph poseGraph(vfile, efile); 
    poseGraph.readGraph(); 

    return 0; 


}