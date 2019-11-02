#include "edge_SE3ProjectXYZ.h"
#include "se3quat.h"
namespace ZM
{
using namespace Eigen;

EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ(const Matrix3d& K):fx(K(0,0)), fy(K(1,1)), cx(K(0,2)), cy(K(1,2))
{



}   

// 观测　- 估计
void EdgeSE3ProjectXYZ::ComputeError()
{
    
    vertexPoint3D*  xyz  =  static_cast< vertexPoint3D*>(Vertexi(0));
    VertexPose6Dof* pose = static_cast< VertexPose6Dof*>(Vertexi(1));

    Vector3d loal_point = pose->Parameters().map(xyz->Parameters());

    Vector2d pixel;
    pixel(0) = fx*loal_point(0) / loal_point(2) + cx;
    pixel(1) = fy*loal_point(1) / loal_point(2) + cy;
    m_error = m_measurement - pixel;

}


void EdgeSE3ProjectXYZ::ComputeJacobian()
{
    vertexPoint3D*  xyz=  static_cast< vertexPoint3D*> (Vertexi(0));
    VertexPose6Dof* pose = static_cast< VertexPose6Dof*>(Vertexi(1));
    Vector3d XYZ = xyz->Parameters();
    SE3Quat T(pose->Parameters());

    Vector3d loal_point = T.map(XYZ);
    double X = loal_point(0);
    double Y = loal_point(1);
    double Zinv = 1.0/loal_point(2);
    double Zinv_2 = Zinv*Zinv;
    

    Matrix<double, 2, 3> tmp;
    tmp(0,0) = - fx * Zinv;
    tmp(0,1) = 0 ;
    tmp(0,2) =  fx * X * Zinv_2;
    tmp(1,0) = 0;
    tmp(1,1) = - fy * Zinv;
    tmp(1,2) = fy * Y * Zinv_2;

    m_jacobianOplusXi = tmp * T.Rotation().toRotationMatrix();

    m_jacobianOplusXj(0,0) = fx * X * Y * Zinv_2;
    m_jacobianOplusXj(0,1) = -(fx + fx*X*X*Zinv_2);
    m_jacobianOplusXj(0,2) = fx * Y * Zinv;
    m_jacobianOplusXj(0,3) = - fx * Zinv;
    m_jacobianOplusXj(0,4) = 0;
    m_jacobianOplusXj(0,5) = fx * X * Zinv_2;
    m_jacobianOplusXj(1,0) = fy + fy * Y * Y * Zinv_2;
    m_jacobianOplusXj(1,1) = -(fy*X*Y*Zinv_2);
    m_jacobianOplusXj(1,2) = -(fy * Zinv * X);
    m_jacobianOplusXj(1,3) = 0;
    m_jacobianOplusXj(1,4) = - fy * Zinv;
    m_jacobianOplusXj(1,5) = fy * Y * Zinv_2;

}



}