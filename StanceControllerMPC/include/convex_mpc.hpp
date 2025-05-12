#ifndef _convex_mpc_hpp_
#define _convex_mpc_hpp_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "OsqpEigen/OsqpEigen.h"
#include <vbmath.hpp>
#include "structs.hpp"
#include "system_functions.hpp"

using namespace Eigen;
using namespace std;

typedef Eigen::SparseMatrix<double> Sparse_Matrix;

#define NUM_LEGS 4
 
class ConvexMPC{
public:
    ConvexMPC();
    ~ConvexMPC();

    void set_mpc_params(double timestep, int horizon, double friction_coeff,
                        double f_min, double f_max, VectorXd &Q, VectorXd &R);
    void set_physical_params(RobotPhysicalParams &robot_params);
    VectorXd get_contact_forces(VectorXd x0, VectorXd x_ref, MatrixXd& foot_positions, vector<int>& contact_state);
    

private:
    
    void calc_AB_matrices(VectorXd& body_rpy, MatrixXd& foot_positions,
                          MatrixXd& A_mat, MatrixXd& B_mat);
    void calc_discrete_matrices(MatrixXd& A_mat, MatrixXd& B_mat, double& planning_timestep,
                                MatrixXd& Ad_mat, MatrixXd& Bd_mat);
    void calc_QP_matrices(int& planning_horizon_steps, VectorXd& x0, VectorXd& state_ref,
                          MatrixXd& Q, MatrixXd& R,
                          MatrixXd& Ad_mat, MatrixXd& Bd_mat, 
                          Sparse_Matrix& H_sparse, VectorXd& q_dense);
    void calc_constraint_matrix(int& planning_horizon_steps, 
                                  Vector4d& friction_coeff, Sparse_Matrix& Ac_sparse);
    void calc_constraint_bounds(int& planning_horizon_steps, vector<int>& contact_state,
                                     double& fz_max, double& fz_min,
                                     VectorXd& l, VectorXd& u);

    void init_solver();
    void update_solver();

    RobotPhysicalParams robot_params;
    VectorXd cur_rpy;

    double dt;
    int horizon;
    Vector4d friction_coeff;
    double f_min;
    double f_max;
    // VectorXd Q, R;
    MatrixXd Qqp, Rqp;

    double cos_yaw;
    double sin_yaw;
    double cos_pitch;
    double tan_pitch;
    int num_legs;
    

    Matrix3d inv_inertia, inv_mass;

    Matrix3d R_xyz;
    MatrixXd R_body;
    MatrixXd Ac_dense;

    MatrixXd A, B;
    MatrixXd Ad, Bd;

    OsqpEigen::Solver solver;

    Sparse_Matrix H;
    VectorXd q;
    Sparse_Matrix Ac;
    VectorXd lb, ub;

    VectorXd ref_grf;
    // bool solver_inited;
    int it_Xref;
    double friction_l_u;

};

#endif //_convex_mpc_hpp_