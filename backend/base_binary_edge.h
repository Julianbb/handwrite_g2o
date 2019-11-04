#pragma once
#include <Eigen/Eigen>
#include <assert.h>
#include <memory>
#include <string>
#include <vector>
#include "base_edge.h"

namespace ZM
{
using namespace Eigen;

class Vertex;



template <int D, typename type_of_mesurement, typename type_of_vertexi, typename type_of_vertexj>
class BaseBinaryEdge : public BaseEdge
{
public:
    typedef std::vector<Vertex*> VertexContainer;
    typedef type_of_mesurement Measurement;
    typedef Matrix<double, D, 1> ErrorVector;
    typedef Matrix<double, D, D> InformationType;
    

    static const int Di = type_of_vertexi::DIMENSION;
    static const int Dj = type_of_vertexj::DIMENSION; // typename::AlignedMapType
    typedef  Matrix<double, D, Di> JacobianXiOplusType;
    typedef  Matrix<double, D, Dj> JacobianXjOplusType;

    BaseBinaryEdge();
 

    const Measurement& GetMeasurement()const{return m_measurement;}
    virtual void  SetMeasurement(const Measurement& m) {m_measurement = m;}

    const InformationType& Information()const{return m_information;}// information matrix can't be modified outside
    InformationType& Information(){return m_information;} // returned information matrix　can be modified outside
    void SetInformation(const InformationType& info){m_information = info;}
    virtual double* InformationData(){return m_information.data();}
    virtual const double* InformationData() const{return m_information.data();}

    ErrorVector& Error() {return m_error;}
    const ErrorVector& Error()const{return m_error;}
    virtual double* ErrorData() {return m_error.data();}
    virtual const double* ErrorData()const {return m_error.data();}


    double Chi2() const 
    {
        return m_error.dot(m_information*m_error);
    }



    const JacobianXiOplusType& jacobianOplusXi() const { return m_jacobianOplusXi;}
    const JacobianXjOplusType& jacobianOplusXj() const { return m_jacobianOplusXj;}

protected:
    using BaseEdge::m_vertices;
    JacobianXiOplusType m_jacobianOplusXi;
    JacobianXjOplusType m_jacobianOplusXj;
    


    Measurement m_measurement;
    ErrorVector m_error;
    InformationType m_information;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "base_binary_edge.hpp"
}