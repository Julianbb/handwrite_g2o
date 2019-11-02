#include <iostream>
#include "vertex.h"
#include "vertex_point.h"
#include "vertex_pose.h"
#include "edge_SE3ProjectXYZ.h"
#include "edge_pose2pose.h"
#include "se3quat.h"



using namespace std;

 

int main()
{
    ZM::vertexPoint3D* v1 = new ZM::vertexPoint3D();
    Eigen::Vector3d xx(1,2,3);
     v1->SetParameters(xx);
     cout << v1->Dimension() << endl;

    ZM::VertexPose6Dof* v2 = new ZM::VertexPose6Dof();
    v2->SetParameters(ZM::SE3Quat());

    ZM::VertexPose6Dof* v3 = new ZM::VertexPose6Dof();
    v3->SetParameters(ZM::SE3Quat());
 
    Eigen::Matrix3d K;
    ZM::EdgeSE3ProjectXYZ* e = new ZM::EdgeSE3ProjectXYZ(K);
    ZM::EdgePose2Pose* e2 = new ZM::EdgePose2Pose();
    e->SetVertex(0, v1);
    e->SetVertex(1, v2);

    ZM::VertexPose6Dof* x = static_cast<ZM::VertexPose6Dof*>(e->Vertexi(1));
    ZM::SE3Quat wo = x->Parameters() ;
    
    cout <<wo.Translation()<< endl;
    cout <<wo.Rotation().toRotationMatrix()<< endl;

    
    e2->SetVertex(0, v2);
    e2->SetVertex(1, v3);
    ZM::VertexPose6Dof* xxx = static_cast<ZM::VertexPose6Dof*>(e2->Vertexi(1));
    ZM::SE3Quat woo = xxx->Parameters() ;
    
    cout <<woo.Translation()<< endl;
    cout <<woo.Rotation().toRotationMatrix()<< endl;

}