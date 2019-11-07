#include "problem.h"
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include "vertex_pose.h"

namespace ZM
{
 Problem::Problem(ProblemType problem_type):m_problem_type(problem_type),m_total_demension(0),
 m_cam_demension(0),m_point_demension(0)
 {

 }


bool IsPoseVertex(Vertex* v)
{
    string type = v->TypeVertex();
    if(type == std::string("VertexPose"))
        return true;
    else
        return false;

}

bool IsLandMarkVertex(Vertex* v)
{
    string type = v->TypeVertex();
    if(type == std::string("VertexPoint"))
        return true;
    else
        return false;

}

void Problem::AddVertex(Vertex* v)
{
    if(m_vertex.find(v->Id()) != m_vertex.end())
        return;
    else
    {
        m_vertex.insert(make_pair(v->Id(), v));
    }
}




void Problem::RemoveVertex(Vertex* v)
{
    if(m_vertex.find(v->Id()) == m_vertex.end())
        return;
    else
    {
        m_vertex.erase(v->Id());
        m_vertex2edge.erase(v->Id());
    }

    // if(IsLandMarkVertex(v))
    // {
    //     m_point_demension -= 3;
    // }
    // else
    // {
    //     m_cam_demension -= 6;
    // }

    //还要　remove vertex 对应的 edge
    vector<BaseEdge*> edges_to_remove;
    edges_to_remove = GetEdgesConnectToVertex(v);
    for(int i=0; i<edges_to_remove.size(); i++)
    {
        RemoveEdge(edges_to_remove[i]);
    }
}




void Problem::AddEdge(BaseEdge* e)
{
    if(m_edge.find(e->Id()) != m_edge.end())
        return;
    else
    {
        m_edge.insert(make_pair(e->Id(),e));
    }

    for(auto vertex : e->Verteies())
    {
        m_vertex2edge.insert(make_pair(vertex->Id(), e));
    }
}


void  Problem::RemoveEdge(BaseEdge* e)
{
    if(m_edge.find(e->Id()) == m_edge.end())
        return;
    else
    {
        m_edge.erase(e->Id());
    }

}


vector<BaseEdge*> Problem::GetEdgesConnectToVertex(Vertex* v)
{
    vector<shared_ptr<BaseEdge>> edges;

    //返回所有m_vertex2edge的 key　是　v-Id()　的元素
    pair<HashVertexId2Edge::iterator, HashVertexId2Edge::iterator> equal_pair = m_vertex2edge.equal_range(v->Id());
    for(auto it = equal_pair.first; it != equal_pair.second; it++)
    {
        if(m_edge.find(it->second->Id()) == m_edge.end()) // 这条边已经被删除
            continue;

        edges.emplace_back(it->second);
    }
}

Vertex* Problem::GetVertex(ulong i)
{
    assert(i<m_vertex.size());
    auto it = m_vertex.find(i);
    if(it!=m_vertex.end())
    {
        return it->second; 
    }
    else
    {
        return nullptr;
    }
}




void Problem::SetOrdering()
{
    unsigned int currentOrder = 0;
    for(auto& v : m_vertex)
    {
        m_total_demension += (v.second)->Dimension(); 
        v.second->SetOrderId(currentOrder);
        m_order_id_vertex.insert(make_pair(currentOrder, v.second));
        currentOrder += (v.second)->Dimension();

        if(IsLandMarkVertex(v.second))
        {
            m_point_demension += (v.second)->Dimension();
        }
        if(IsPoseVertex(v.second))
        {
            m_cam_demension += (v.second)->Dimension();
        }
    }
}






void Problem::MakeHessian()
{
    ulong H_size = m_total_demension;
    MatrixXd H = MatrixXd::Zero(H_size, H_size);
    VectorXd b = VectorXd::Zero(H_size, 1);


    for(auto& edge: m_edge)
    {
        edge.second->ComputeError();
        edge.second->ComputeJacobian();


        auto jacobians = edge.second->Jacobians();
        auto verteies = edge.second->Verteies();


        assert(jacobians.size() == verteies.size());

        for(int i=0; i<verteies.size(); i++)
        {
            auto v_i = verteies[i];
            if(v_i->IsFixed())
                continue;
            
            auto jacobiani = jacobians[i];
            ulong index_i = v_i->OrderId();
            ulong dim_i = v_i->Dimension();

            Eigen::Map<Matrix<double, 6, 6>> info_matrix(edge.second->InformationData()); // 信息矩阵

            MatrixXd Jt = jacobiani.transpose()*info_matrix;
            
            for(int j=i; j<verteies.size(); j++)
            {
                auto v_j = verteies[j];
                if(v_j->IsFixed())
                    continue;
                
                auto jacobianj = jacobians[j];
                ulong index_j = v_j->OrderId();
                ulong dim_j = v_j->Dimension();

                MatrixXd Hessian = Jt * jacobianj;

                H.block(index_i, index_j, dim_i, dim_j).noalias() += Hessian;
                if(i!=j) //对称
                {
                    H.block(index_j, index_i, dim_j, dim_i).noalias() += Hessian.transpose();
                }
            }
            VectorXd::ConstMapType tmp_b (edge.second->ErrorData(), edge.second->Dimension());
            b.segment(index_i, dim_i).noalias() -= Jt*tmp_b;
        }
    }


    // TODO: Ax = b, 如果A的行列式为0, A不可逆，LM算法每次解方程对角＋lamda,保证矩阵可逆
    m_H = H;
    m_b = b;
   
    m_deltax = VectorXd::Zero(H_size);
}



// void Problem::MakeHessian()
// {
//     ulong H_size = m_total_demension;
//     ulong edge_size = m_edge.size();
//     MatrixXd J = MatrixXd::Zero(edge_size*6, H_size);
//     VectorXd b = VectorXd::Zero(edge_size*6, 1);

//     int index_edge = 0;
//     for(auto& edge: m_edge)
//     {
//         // VertexPose6Dof* v11 = static_cast<VertexPose6Dof*>(edge.second->Vertexi(0));
//         // VertexPose6Dof* v22 = static_cast<VertexPose6Dof*>(edge.second->Vertexi(1));

//         edge.second->ComputeError();
//         edge.second->ComputeJacobian();

//         auto jacobians = edge.second->Jacobians();
//         auto verteies = edge.second->Verteies();

//         assert(jacobians.size() == verteies.size());


//         auto v_i = verteies[0];
//         // if(v_i->IsFixed())
//         //     continue;
        
//         auto jacobiani = jacobians[0];
//         ulong index_i = v_i->OrderId();
//         ulong dim_i = v_i->Dimension();

//         auto v_j = verteies[1];
//         // if(v_j->IsFixed())
//         //     continue;
        
//         auto jacobianj = jacobians[1];
//         ulong index_j = v_j->OrderId();
//         ulong dim_j = v_j->Dimension();

//         J.block(index_edge, index_i, 6, 6).noalias() = jacobiani;
//         J.block(index_edge, index_j, 6, 6).noalias() = jacobianj;
        
    
//         VectorXd::ConstMapType tmp_b (edge.second->ErrorData(), edge.second->Dimension());
//         b.segment(index_edge, 6) = tmp_b;

//         index_edge += 6;   //只针对 pose_graph
//     }
    
//     // TODO: Ax = b, 如果A的行列式为0, A不可逆，怎么解决????
//     MatrixXd Jt = J.transpose();
//     m_H = Jt*J;
//     cout << Jt.cols() <<endl;
//     cout << b.rows() << endl;
//     m_b = -Jt*b;
     
//     m_deltax = VectorXd::Zero(H_size);
// }




void Problem::InitLMParameters() 
{
    m_lambda = 1.0;
    m_ni = 2.0;
    m_chi2 = 0.;

    for(auto& e : m_edge)
    {
        m_chi2 += e.second->Chi2();
    }
    m_stop_chi2 = 1e-6 * m_chi2;  //误差下降了1e-6倍，则停止


    double maxDiagnal = 0.;
    ulong size = m_H.rows();
    for(int i = 0; i <size; i++)
        maxDiagnal = std::max(std::fabs(m_H(i,i)), maxDiagnal);

    double tau = 1e-5;
    m_lambda = tau*maxDiagnal;  // lamda初始值　= 1e-5 * H对角元素的最大值

}


bool Problem::IsGoodStepInLM()
{
    double scale = 0.;
    scale = 0.5 * m_deltax.transpose()*(m_lambda*m_deltax + m_b);
    scale += 1e-3; // 防止为０


    double tmp_chi2 =0.;
    for(auto& e:m_edge)
    {
        e.second->ComputeError();
        tmp_chi2 += e.second->Chi2();
    }

    double rho = (m_chi2 - tmp_chi2) / scale;
    if(rho > 0) //证明上次的迭代误差是下降的
    {
        double tmp_v = 1 - std::pow(2*rho-1, 3);
        //double tmp_m = max();
        double tmp_factor = std::max(1./3, tmp_v);
        
        m_lambda = m_lambda * tmp_factor;
        m_ni = 2.0;

        m_chi2 = tmp_chi2; //跟新误差
        return true;
    }
    else
    {
        m_lambda = m_lambda * m_ni;
        m_ni = 2*m_ni;
        return false;
    }

}


void Problem::RemoveLambdaHessianLM()
{
    ulong size = m_H.rows();
    assert(m_H.rows() == m_H.cols());

    for(int i = 0; i <size; i++)
    {
        m_H(i,i) += m_lambda;
    }

}


void Problem::AddLambdatoHessianLM()
{
    ulong size = m_H.rows();
    assert(m_H.rows() == m_H.cols());

    for(int i = 0; i <size; i++)
    {
        m_H(i,i) -= m_lambda;
    }
}

//================================================================
//           |      |      |      |     |
//           |  Hpp |  Hpm |      | bpp |
//           |      |      |      |     |
//          ------------------    |     |
//           |      |      |      |     |
//           |  Hmp |  Hmm |      | bmm |
//           |      |      |      |     |
//================================================================
void Problem::SolveLinearSystem()
{
    if(m_problem_type == SLAM_PROBLEM) // SLAM 问题采用舒尔补的计算方式
    {
        int reserve_size = m_cam_demension;
        int marg_size = m_point_demension;

        MatrixXd Hmm = m_H.block(reserve_size, reserve_size, marg_size, marg_size);
        MatrixXd Hpp = m_H.block(0,  0,  reserve_size, reserve_size);
        MatrixXd Hpm = m_H.block(0, reserve_size, reserve_size, marg_size);
        MatrixXd Hmp = m_H.block(reserve_size, 0, marg_size, reserve_size);

        VectorXd bpp = m_b.segment(0, reserve_size);
        VectorXd bmm = m_b.segment(reserve_size, marg_size);
        
        MatrixXd Hmm_inv = MatrixXd::Zero(marg_size, marg_size);
        int num_of_points = m_point_demension/3; //偷懒写法，这里只针对3d点，比如其他参数化点:逆深度不适用
        for(int i=0; i<num_of_points; i++)
        {
            Hmm_inv.block(i, i, 3, 3) = Hmm.block(i, i, 3, 3).inverse();
        }

        MatrixXd tmp_H = Hpp - Hpm * Hmm_inv * Hmp;
        MatrixXd tmp_b = bpp - Hpm * Hmm_inv * bmm;


        for(int i=0; i<m_cam_demension; i++)
        {
            tmp_H(i,i) += m_lambda;
        }
        VectorXd tmp_delta_Xc = VectorXd::Zero(reserve_size);
        int iterators = tmp_H.rows() *2 ; //迭代次数
        tmp_delta_Xc = PCGSolver(tmp_H, tmp_b, iterators);
        m_deltax.head(reserve_size) = tmp_delta_Xc;

        VectorXd tmp_delta_Xp = Hmm_inv * (bmm - Hmp* tmp_delta_Xc);
        m_deltax.tail(marg_size) = tmp_delta_Xp;
    }

    if(m_problem_type == GENERAL_PROBLEM)
    {

        MatrixXd tmp_H = m_H;
        for(int i = 0; i <m_H.rows(); i++) // 解方程对角线+lamda
            tmp_H(i,i) += m_lambda;
       //m_deltax = PCGSolver(tmp_H, m_b, m_H.rows() * 2); //TODO PCGSolver根本就没改变结果???
        //m_deltax = tmp_H.inverse() * m_b;
        m_deltax = tmp_H.llt().solve(m_b);
    }

}

void Problem::RollbackStates()
{
    for(auto& v : m_vertex)
    {
        int index = v.second->OrderId();
        int dim = v.second->Dimension();
        VectorXd x_part = m_deltax.segment(index, dim);
        x_part = -x_part;
        v.second->Plus(&x_part[0]); //减去　delta_x
    }
}

void Problem::UpdateStates()
{
    for(auto& v : m_vertex)
    {
        if(v.second->IsFixed())
            continue;
        int index = v.second->OrderId();
        int dim = v.second->Dimension();
        VectorXd x_part = m_deltax.segment(index, dim);
        v.second->Plus(&x_part[0]); 
    }
}



bool Problem::Solve(int itrators)
{
    if(m_edge.size() == 0 || m_vertex.size() == 0)
    {
        std::cerr << "no edge or vertices"  << std::endl;
        return false;
    }

    SetOrdering();
    MakeHessian();
    InitLMParameters();

    bool stop = false;
    int its = 0; 

    while(!stop && (its < itrators))
    {
        bool oneStepSucceed = false;
        int false_cnt = 0;

        std::cout << "iter: " << its << " , chi= " << m_chi2 << " , Lambda= " << m_lambda  << " false_cnt: "  << false_cnt << std::endl;

        while(!oneStepSucceed)
        {
            SolveLinearSystem();

            if(m_deltax.squaredNorm() < 1e-6 || false_cnt > 10)
            {
                stop = true;
                break;
            }
            UpdateStates();
            oneStepSucceed = IsGoodStepInLM();

            if(oneStepSucceed)
            {
                MakeHessian();
                false_cnt = 0;
            }
            else
            {
                false_cnt++;
                RollbackStates(); // 误差没下降，回滚
            }
        }

        its++;
        if(m_chi2 < m_stop_chi2)
            stop = true;

    }

    return true;
}







/** @brief conjugate gradient with perconditioning
 *
 *  the jacobi PCG method
 *
 */
VectorXd Problem::PCGSolver(const MatrixXd &A, const VectorXd &b, int maxIter)
{
    assert(A.rows() == A.cols() && "PCG solver ERROR: A is not a square matrix");
    int rows = b.rows();
    int n = maxIter < 0 ? rows : maxIter;
    VectorXd x(VectorXd::Zero(rows));
    MatrixXd M_inv = A.diagonal().asDiagonal().inverse();
    VectorXd r0(b); // initial r = b - A*0 = b
    VectorXd z0 = M_inv * r0;
    VectorXd p(z0);
    VectorXd w = A * p;
    double r0z0 = r0.dot(z0);
    double alpha = r0z0 / p.dot(w);
    VectorXd r1 = r0 - alpha * w;
    int i = 0;
    double threshold = 1e-6 * r0.norm();
    while (r1.norm() > threshold && i < n)
    {
        i++;
        VectorXd z1 = M_inv * r1;
        double r1z1 = r1.dot(z1);
        double belta = r1z1 / r0z0;
        z0 = z1;
        r0z0 = r1z1;
        r0 = r1;
        p = belta * p + z1;
        w = A * p;
        alpha = r1z1 / p.dot(w);
        x += alpha * p;
        r1 -= alpha * w;
    }
    
    // cout << x << endl;
    // cout << x.squaredNorm() << endl;
    return x;
}


}