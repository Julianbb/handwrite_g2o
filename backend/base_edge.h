#pragma once
#include <Eigen/Eigen>
#include "vertex.h"


namespace ZM
{
using namespace Eigen;

class Vertex;





template <int D, typename type_of_mesurement>
class BaseEdge
{
public:
    typedef type_of_mesurement Measurement;
    typedef Matrix<double, D, 1> ErrorVector;
    typedef Matrix<double, D, D> InformationType;
    typedef std::vector<Vertex*> VertexContainer;

    const InformationType& Information()const{return m_information;}// information matrix can't be modified outside
    InformationType& Information(){return m_information;} // returned information matrix　can be modified outside
    void SetInformation(const InformationType& info){m_information = info;}
    virtual double* InformationData(){return m_information.data();}
    virtual const double* InformationData() const{return m_information.data();}

    const Measurement& GetMeasurement()const{return m_measurement;}
    virtual void  SetMeasurement(const Measurement& m) {m_measurement = m;}


    double Chi2() const 
    {
        return m_error.dot(m_information*m_error);
    }

    ErrorVector& Error() {return m_error;}
    const ErrorVector& Error()const{return m_error;}
    virtual double* ErrorData() {return m_error.data();}
    virtual const double* ErrorData()const {return m_error.data();}

    void SetId(const unsigned int& id){m_id = id;}
    unsigned int Id(){return m_id;}

protected:
    Measurement m_measurement;
    ErrorVector m_error;
    InformationType m_information;
    unsigned int m_id;

    VertexContainer m_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}