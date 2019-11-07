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

    vector<VertexPose6Dof*> vertex_pose;
    vector<EdgePose2Pose*>  edge_pose;

    Eigen::AngleAxisd R(M_PI/4, Eigen::Vector3d(0,0,1));
    Eigen::AngleAxisd R1(M_PI/2, Eigen::Vector3d(0,0,1));

    Problem problem(Problem::GENERAL_PROBLEM);

    VertexPose6Dof* v1 = new VertexPose6Dof();
    int index = 0; 
    v1->SetId(index);
    v1->SetParameters(SE3Quat(Eigen::Quaterniond(1,0,0,0),
                Eigen::Vector3d(0,0,0)));
    v1->SetFixed(true);
    problem.AddVertex(v1);
    //v1->SetFixed(true);

    VertexPose6Dof* v2 = new VertexPose6Dof();
    index++; 
    v2->SetId(index);
    v2->SetParameters(SE3Quat(Eigen::Quaterniond(R.toRotationMatrix()),
                Eigen::Vector3d(0,0,0)));
    problem.AddVertex(v2);
    

    EdgePose2Pose* e = new EdgePose2Pose();
    e->SetId(0);
    e->SetVertex(0, v1);
    e->SetVertex(1, v2);
    e->SetMeasurement(SE3Quat(Eigen::Quaterniond(R1.toRotationMatrix()),
                Eigen::Vector3d(0,0,0)));
    problem.AddEdge(e);


    cout << "===================== before optimization =================" << endl;


    cout << v1->Parameters().Rotation().coeffs().transpose() << endl;
    cout << v1->Parameters().Translation().transpose() << endl;

    cout << v2->Parameters().Rotation().coeffs().transpose() << endl;
    cout << v2->Parameters().Translation().transpose() << endl;


    problem.Solve(3);



    cout << "===================== after optimization =================" << endl;
    VertexPose6Dof* result1 = static_cast<VertexPose6Dof*>(problem.GetVertex(0));
    VertexPose6Dof* result2 = static_cast<VertexPose6Dof*>(problem.GetVertex(1));

    cout << result1->Parameters().Rotation().coeffs().transpose() << endl;
    cout << result1->Parameters().Translation().transpose() << endl;

    cout << result2->Parameters().Rotation().coeffs().transpose() << endl;
    cout << result2->Parameters().Translation().transpose() << endl;

}


