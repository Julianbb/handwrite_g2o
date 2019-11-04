
#pragma once
#include <Eigen/Eigen>
#include <assert.h>

namespace ZM
{
using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;



class SE3Quat
{
public:
    SE3Quat()
    {
        m_t.setZero();
        m_q.setIdentity();
    }

    SE3Quat(const Matrix3d& R, const Vector3d& t):m_q(Quaterniond(R)), m_t(t)
    {
        normalizeRotation();
    }
    SE3Quat(const Quaterniond& q, const Vector3d& t):m_q(q),m_t(t)
    {
        normalizeRotation();
    }


    inline const Vector3d& Translation() const {return m_t;}
    inline void SetTranslation(const Vector3d& t){m_t=t;}
    inline const Quaterniond& Rotation() const {return m_q;}
    inline void SetRotation(const Quaterniond& q){m_q=q;}


    inline SE3Quat operator*(const SE3Quat& other) const
    {
        SE3Quat result = (*this);
        result.m_q = this->m_q* other.m_q;
        result.m_t = this->m_t + this->m_q * other.m_t;

        result.normalizeRotation();
        return result;
    }


    inline SE3Quat& operator*=(const SE3Quat& other) 
    {
        m_t += m_q * other.m_t;
        m_q *= other.m_q;

        normalizeRotation();
        return *this;
    }

    inline Vector3d operator*(const Vector3d& other) const
    {
        return m_q*other + m_t;
    }

    inline SE3Quat inverse() const
    {
        SE3Quat result;
        
        result.m_q = m_q.conjugate();
        result.m_t = result.m_q * ( m_t * -1.);
        return result;
    }



    inline double operator[](int i)
    {
        assert(i < 7);
        if(i<3)
            return m_t[i];
        else
            return m_q.coeffs()[i-3];
    }


    inline Vector7d toVector() const{
    Vector7d v;
    v[0]=m_t(0);
    v[1]=m_t(1);
    v[2]=m_t(2);
    v[3]=m_q.x();
    v[4]=m_q.y();
    v[5]=m_q.z();
    v[6]=m_q.w();
    return v;
    }


    inline void fromVector(const Vector7d& v)
    {
       m_q = Quaterniond(v[6], v[3], v[4], v[5]); 
       m_t = Vector3d(v[0], v[1], v[2]);
    }


    inline Vector6d toMinimalVector() const{
    Vector6d v;
    v[0]=m_t(0);
    v[1]=m_t(1);
    v[2]=m_t(2);
    v[3]=m_q.x();
    v[4]=m_q.y();
    v[5]=m_q.z();
    return v;
    }


    inline void fromMinimalVector(const Vector6d& v){
    double w = 1.-v[3]*v[3]-v[4]*v[4]-v[5]*v[5];
    if (w>0){
        m_q=Quaterniond(sqrt(w), v[3], v[4], v[5]);
    } else {
        m_q=Quaterniond(0, -v[3], -v[4], -v[5]);
    }
    m_t=Vector3d(v[0], v[1], v[2]);
    }


 Vector3d deltaR(const Matrix3d& R) const
  {
    Vector3d v;
    v(0)=R(2,1)-R(1,2);
    v(1)=R(0,2)-R(2,0);
    v(2)=R(1,0)-R(0,1);
    return v;
  }


    //旋转在前　平移在后
    Vector6d log() const {
    Vector6d res;
    Matrix3d _R = m_q.toRotationMatrix();
    double d =  0.5*(_R(0,0)+_R(1,1)+_R(2,2)-1);
    Vector3d omega;
    Vector3d upsilon;


    Vector3d dR = deltaR(_R);
    Matrix3d V_inv;

    if (d>0.99999)
    {

        omega=0.5*dR;
        Matrix3d Omega = SO3hat(omega);
        V_inv = Matrix3d::Identity()- 0.5*Omega + (1./12.)*(Omega*Omega);
    }
    else
    {
        double theta = acos(d);
        omega = theta/(2*sqrt(1-d*d))*dR;
        Matrix3d Omega = SO3hat(omega);
        V_inv = ( Matrix3d::Identity() - 0.5*Omega
            + ( 1-theta/(2*tan(theta/2)))/(theta*theta)*(Omega*Omega) );
    }

    upsilon = V_inv*m_t;
    for (int i=0; i<3;i++){
        res[i]=omega[i];
    }
    for (int i=0; i<3;i++){
        res[i+3]=upsilon[i];
    }

    return res;

    }

    Vector3d map(const Vector3d & xyz) const
    {
    return m_q*xyz + m_t;
    }


    static SE3Quat exp(const Vector6d & update)
    {
        Vector3d omega;
        for (int i=0; i<3; i++)
        omega[i]=update[i];
        Vector3d upsilon;
        for (int i=0; i<3; i++)
        upsilon[i]=update[i+3];

        double theta = omega.norm();
        Matrix3d Omega = SO3hat(omega);

        Matrix3d R;
        Matrix3d V;
        if (theta<0.00001)
        {
        //TODO: CHECK WHETHER THIS IS CORRECT!!!
        R = (Matrix3d::Identity() + Omega + Omega*Omega);

        V = R;
        }
        else
        {
        Matrix3d Omega2 = Omega*Omega;

        R = (Matrix3d::Identity()
            + sin(theta)/theta *Omega
            + (1-cos(theta))/(theta*theta)*Omega2);

        V = (Matrix3d::Identity()
            + (1-cos(theta))/(theta*theta)*Omega
            + (theta-sin(theta))/(pow(theta,3))*Omega2);
        }
        return SE3Quat(Quaterniond(R),V*upsilon);
    }


//和slam十四讲不一样，李代数旋转平移顺序不一样
    Matrix<double, 6, 6> adj() const
    {
        Matrix3d R = m_q.toRotationMatrix();
        Matrix<double, 6, 6> res;
        res.block(0,0,3,3) = R;
        res.block(3,3,3,3) = R;
        res.block(3,0,3,3) = SO3hat(m_t)*R;
        res.block(0,3,3,3) = Matrix3d::Zero(3,3);
        return res;
    }


    Matrix<double,4,4> to_homogeneous_matrix() const
    {
        Matrix<double,4,4> homogeneous_matrix;
        homogeneous_matrix.setIdentity();
        homogeneous_matrix.block(0,0,3,3) = m_q.toRotationMatrix();
        homogeneous_matrix.col(3).head(3) = Translation();

        return homogeneous_matrix;
    }

    void normalizeRotation()
    {
        if(m_q.w() < 0)
        {
            m_q.coeffs() *= -1;
        }
        m_q.normalize();
    }

    Isometry3d toIsometry3d() const
    {
        Eigen::Isometry3d result = (Eigen::Isometry3d) Rotation();
        result.translation() = Translation();
        return result;
    }




    
    static Matrix3d SO3hat(const Vector3d & v)
    {
        Matrix3d Omega;
        Omega <<  0, -v(2),  v(1)
            ,  v(2),     0, -v(0)
            , -v(1),  v(0),     0;
        return Omega;
    }







// right jacobian of SO(3)
static Matrix3d SO3JacobianR(const Vector3d& w)
{
    Matrix3d Jr = Matrix3d::Identity();
    double theta = w.norm();
    if(theta<0.00001)
    {
        return Jr;// = Matrix3d::Identity();
    }
    else
    {
        Vector3d k = w.normalized();  // k - unit direction vector of w
        Matrix3d K = SO3hat(k);
        Jr =   Matrix3d::Identity()
                - (1-cos(theta))/theta*K
                + (1-sin(theta)/theta)*K*K;
    }
    return Jr;
}
static Matrix3d SO3JacobianRInv(const Vector3d& w)
{
    Matrix3d Jrinv = Matrix3d::Identity();
    double theta = w.norm();

    // very small angle
    if(theta < 0.00001)
    {
        return Jrinv;
    }
    else
    {
        Vector3d k = w.normalized();  // k - unit direction vector of w
        Matrix3d K = SO3hat(k);
        Jrinv = Matrix3d::Identity()
                + 0.5*SO3hat(w)
                + ( 1.0 - (1.0+cos(theta))*theta / (2.0*sin(theta)) ) *K*K;
    }

    return Jrinv;
}

// left jacobian of SO(3), Jl(x) = Jr(-x)
static Matrix3d SO3JacobianL(const Vector3d& w)
{
    return SO3JacobianR(-w);
}
// left jacobian inverse
static Matrix3d SO3JacobianLInv(const Vector3d& w)
{
    return SO3JacobianRInv(-w);
}
// ---------------------------------


protected:
    Vector3d m_t;
    Quaterniond m_q;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};



  inline std::ostream& operator << (std::ostream& out_str, const SE3Quat& se3)
  {
    out_str << se3.to_homogeneous_matrix()  << std::endl;
    return out_str;
  }

}