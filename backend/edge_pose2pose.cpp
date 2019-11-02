#include "edge_pose2pose.h"


namespace ZM
{


void EdgePose2Pose::ComputeError()
{
    VertexPose6Dof* pose1 = static_cast< VertexPose6Dof*>(Vertexi(0));
    VertexPose6Dof* pose2 = static_cast< VertexPose6Dof*>(Vertexi(1));
    SE3Quat estimate_i = pose1->Parameters();
    SE3Quat estimate_j = pose2->Parameters();


    SE3Quat estimate_ij = estimate_i.inverse()* estimate_j;
    Isometry3d tmp_ji = m_measurement.inverse();
    SE3Quat measurement_ji = SE3Quat(tmp_ji.rotation(), tmp_ji.translation());

    SE3Quat error = measurement_ji * estimate_ij;
            
    m_error = error.log(); ////旋转在前　平移在后
}



// TODO: 是否正确，可能和14讲旋转　平移定义的顺序不同
Matrix6d EdgePose2Pose::JRInv(Vector6d error)
{
    Vector3d R = Vector3d(error(0), error(1), error(2));
    Vector3d fake_t = Vector3d(error(3), error(4), error(5));
    //Vector3d true_t = SE3Quat::SO3JacobianL(fake_t);   //TODO有问题

     Matrix6d J;
    // J.block<3, 3>(0, 0) = SE3Quat::SO3hat(R); 
    // J.block<3, 3>(0, 3) = SE3Quat::SO3hat(true_t);
    // J.block<3, 3>(3, 0) = Matrix3d::Zero();
    // J.block<3, 3>(3, 3) = SE3Quat::SO3hat(Vector3d(R));

    // J = J*0.5 + Matrix6d::Identity();
    //J = Matrix6d::Identity();// TODO:　try this one

    return J;
}

void EdgePose2Pose::ComputeJacobian()
{
    VertexPose6Dof* pose1 = static_cast< VertexPose6Dof*>(Vertexi(0));
    VertexPose6Dof* pose2 = static_cast< VertexPose6Dof*>(Vertexi(1));
    SE3Quat estimate_i = pose1->Parameters();
    SE3Quat estimate_j = pose2->Parameters();

     Matrix6d J = JRInv(m_error);

     m_jacobianOplusXi = -J * estimate_j.inverse().adj();

     m_jacobianOplusXj = J * estimate_j.inverse().adj();

}






}