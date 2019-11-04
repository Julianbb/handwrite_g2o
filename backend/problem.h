#pragma once
#include <Eigen/Eigen>
#include <map>
#include <unordered_map>
#include <memory>
#include <vector>
#include "vertex.h"
#include "base_edge.h"


namespace ZM
{
    using namespace std;
    using namespace Eigen;

class Problem
{
public:

    enum  ProblemType
    {
        SLAM_PROBLEM, // slam问题
        GENERAL_PROBLEM // 一般问题
    };

    typedef unsigned long ulong;
    typedef map<ulong, shared_ptr<Vertex>> HashVertex;
    typedef unordered_map<ulong, shared_ptr<BaseEdge>> HashEdge;
    typedef unordered_multimap<ulong, shared_ptr<BaseEdge>> HashVertexId2Edge;

    Problem(ProblemType problem_type);
    
    void AddVertex(shared_ptr<Vertex> v);
    void AddEdge(shared_ptr<BaseEdge> e);
    void RemoveVertex(shared_ptr<Vertex> v);
    void RemoveEdge(shared_ptr<BaseEdge> e);
    vector<shared_ptr<BaseEdge>> GetEdgesConnectToVertex(shared_ptr<Vertex> v);
    shared_ptr<Vertex> GetVertex(ulong i);
    

    void SetOrdering();

    bool Solve(int itrators);
    void MakeHessian();
    void InitLMParameters(); 
    void RemoveLambdaHessianLM();
    void AddLambdatoHessianLM();
    bool IsGoodStepInLM();
    void SolveLinearSystem();
    void RollbackStates();
    void UpdateStates();


    VectorXd PCGSolver(const MatrixXd &A, const VectorXd &b, int maxIter = -1);

protected:
    ProblemType m_problem_type;

    HashVertex m_vertex;  
    // HashVertex m_pose_vertex; 
    // HashVertex m_point_vertex;
    HashEdge   m_edge;
    HashVertexId2Edge m_vertex2edge; // 由顶点查询边


    ulong m_total_demension;  //总变量维度　m_total_demension=m_cam_demension+m_point_demension
    ulong m_cam_demension; // 总相机的维度
    ulong m_point_demension; //总３d点的维度
    HashVertex m_order_id_vertex;// <order_id, vertex>

private:
    Matrix<double, Dynamic, Dynamic> m_H;
    Matrix<double, Dynamic, 1>       m_b;
    Matrix<double, Dynamic, 1>       m_deltax;


    double m_ni; //控制阻尼因子的增长
    double m_lambda; //阻尼因子
    double m_chi2; // 当前误差方
    double m_stop_chi2;//


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




}