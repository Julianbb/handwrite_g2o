#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include "vertex.h"
#include "vertex_point.h"
#include "vertex_pose.h"
#include "edge_SE3ProjectXYZ.h"
#include "edge_pose2pose.h"
#include "se3quat.h"
#include "problem.h"

using namespace std;
using namespace Eigen;
using namespace ZM;
 

int main(int argc, char* argv[])
{
   if(argc < 2)
   {
       std::cout << "wrong usage" << endl;
       return -1;
   }

    std::string file = argv[1];
    ifstream fin(file.c_str());
    if(!fin)
    {
        std::cout << "could not open file" << endl;
        return -2;
    }

    int vertex_cnt = 0, edge_cnt = 0;
    vector<VertexPose6Dof*> vertex_pose;
    vector<EdgePose2Pose*>  edge_pose;


    Problem problem(Problem::GENERAL_PROBLEM);
    while(!fin.eof())
    {
        string name;
        fin >> name;

        if(name == "VERTEX_SE3:QUAT")
        {
            VertexPose6Dof* v = new VertexPose6Dof();
            int index = 0; 
            fin >> index;
            v->SetId(index);
            v->read(fin);
            
            if(index == 0)
                v->SetFixed(true);
            problem.AddVertex(v);
            vertex_cnt++;
            vertex_pose.push_back(v);
        }
        else if(name == "EDGE_SE3:QUAT")
        {
            EdgePose2Pose* e = new EdgePose2Pose();
            int index_i, index_j;
            fin >> index_i >> index_j;
            e->SetId(edge_cnt);
            Vertex* xx = problem.GetVertex(index_i);
            e->SetVertex(0, problem.GetVertex(index_i));
            e->SetVertex(1, problem.GetVertex(index_j));

            e->read(fin);
            problem.AddEdge(e);
            edge_pose.push_back(e);
            edge_cnt++;
        }
    }
    fin.close();


    std::cout << "total vertices " << vertex_cnt << endl;
    std::cout << "total edges " << edge_cnt << endl;
    std::cout << "preparing optimizing" << endl;


    problem.Solve(30);

    std::cout << "saving optimizing result" << endl;
    ofstream fout("result.g2o");
    for(auto& v:vertex_pose)
    {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for(auto& e: edge_pose)
    {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout);
    }

    fout.close();
}
