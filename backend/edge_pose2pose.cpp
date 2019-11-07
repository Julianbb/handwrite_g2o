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
    //Isometry3d tmp_ji = m_measurement.inverse();
    //SE3Quat measurement_ji = SE3Quat(tmp_ji.rotation(), tmp_ji.translation());
    SE3Quat measurement_ji =m_measurement.inverse();
    SE3Quat error = measurement_ji * estimate_ij;
            
    m_error = error.log(); ////旋转在前　平移在后
    Vector3d t = m_error.tail<3>();
    m_error.tail<3>() = m_error.head<3>();
    m_error.head<3>() = t; // 平移在前，旋转在后
}




Matrix6d EdgePose2Pose::JRInv(Vector6d error)
{
    Vector3d fake_t = Vector3d(error(0), error(1), error(2));
    Vector3d R = Vector3d(error(3), error(4), error(5));
    

    
     Matrix6d J;
     
     J.block<3, 3>(0, 0) = SE3Quat::SO3hat(R); 
     J.block<3, 3>(0, 3) = SE3Quat::SO3hat(fake_t);
     J.block<3, 3>(3, 0) = Matrix3d::Zero();
     J.block<3, 3>(3, 3) = SE3Quat::SO3hat(R);

    J = J*0.5 + Matrix6d::Identity();
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


     m_jacobians[0] = m_jacobianOplusXi;
     m_jacobians[1] = m_jacobianOplusXj;

}


void EdgePose2Pose::read(ifstream& is)
{
    double data[7];
    for (int i=0; i<7; i++)
        is >> data[i];
    
    Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
    q.normalize();
    SE3Quat tmp_pose(q, Vector3d(data[0], data[1], data[2]));
    SetMeasurement(tmp_pose);



    for(int i=0; i<Information().rows() && is.good(); i++)
    {
        for(int j=i; j<Information().cols() && is.good(); j++)
        {
            is >> Information()(i,j);
            if(i!=j)
                Information()(j,i) = Information()(i,j);
        }
    }

    // Matrix3d xx1 = Information().topLeftCorner(3,3);
    // Information().topLeftCorner(3,3) = Information().bottomRightCorner(3,3);
    // Information().bottomRightCorner(3,3) = xx1;
    

}


void EdgePose2Pose::write(ofstream& os)
{
    VertexPose6Dof* pose1 = static_cast< VertexPose6Dof*>(Vertexi(0));
    VertexPose6Dof* pose2 = static_cast< VertexPose6Dof*>(Vertexi(1));

    os << pose1->Id() << " " << pose2->Id() << " ";
    SE3Quat m = m_measurement;
    Quaterniond q = m.Rotation().normalized();
    os << m.Translation().transpose() << " ";
    os << q.coeffs()[0]<< " "
        << q.coeffs()[1]<< " "
        << q.coeffs()[2]<< " "
        << q.coeffs()[3]<< endl;


    for(int i=0; i<Information().rows(); i++)
    {
        for(int j=i; j<Information().cols(); j++)
        {
            os << Information()(i,j) << " ";
        }
    } 
    os << endl;
}



}