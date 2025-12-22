# ifndef MPS_HPP
# define MPS_HPP

# include <Eigen/Eigen>
#include <Eigen/Sparse>
# include <iostream>
# include <vector>
#include "OsqpEigen/OsqpEigen.h"

namespace mps
{
    Eigen::MatrixXd fibonacci_sphere(int num_points) 
    {
        std::cout << "fibonacci_sphere()." << std::endl;
        if (num_points <= 0) 
        {
            return Eigen::MatrixXd(0, 3);
        }
        Eigen::MatrixXd points(num_points, 3);
        const double golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;
        const double angle_increment = 2.0 * M_PI / golden_ratio;
        for (int i = 0; i < num_points; ++i) {
            double y = 1.0 - (2.0 * i) / (num_points - 1.0);
            double radius = std::sqrt(std::max(0.0, 1.0 - y * y));
            double theta = i * angle_increment;
            points(i, 0) = radius * std::cos(theta); // x
            points(i, 1) = y;                        // y
            points(i, 2) = radius * std::sin(theta); // z
        }
        return points;
    }


    // Check if the polytope is feasible for the given obstacles and minimum distance requirement
    // Obstacles in the same shape (box)
    inline bool check_polytope_feasibility(
        const Eigen::MatrixX3d A,
        const Eigen::VectorXd b, // polytope in H-rep (Ax<=b) format: [A -b]
        const std::vector<Eigen::MatrixX4d> &obstacles, // [O -o] list
        const double &d_min,
        OsqpEigen::Solver &solver) // minimum distance requirement
    {
    
        // initialize the solver
        // objective: 0
        // variables: lambda, mu, i.e., x = [lambda; mu]
        // s.t. 
        // lambda >= 0
        // mu >= 0
        // - o^T *lambda -b^T *mu >= d_min
        //  O^T *lambda + A^T *mu = 0
        // (O* v_k)^T * lambda <= 1 , k = 1, ..., N_k
        std::cout << "check_polytope_feasibility()." << std::endl;
        // solver.settings()->setWarmStart(true);
        static bool first_call = true;
        solver.clearSolver();
        int m_A = A.rows();
        int n_mu = A.rows();
        int m_O = 6;
        int n_lambda = 6;
        int dim = n_lambda + n_mu; 
        int n_v = 20;
        int n_constraints = 0;
        Eigen::MatrixXd V(n_v, 3);
        V = fibonacci_sphere(n_v);
        // if m_O fixed.
        dim = m_A + m_O;
        solver.data()->setNumberOfVariables(dim);
        // cost function: 0
        Eigen::SparseMatrix<double> hessian(dim, dim);
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dim);
        n_constraints = m_O + m_A + 1 + 3 + n_v;
        // Set the l <= A_c*x <= u
        Eigen::SparseMatrix<double> linearMatrix(n_constraints, dim);
        Eigen::VectorXd lowerBound = Eigen::VectorXd::Constant(n_constraints, -1.0e9);
        Eigen::VectorXd upperBound = Eigen::VectorXd::Constant(n_constraints, 1.0e9);
        std::vector<Eigen::Triplet<double>> tripletList;
        if (first_call)
        {
            first_call = false;
            solver.data()->setHessianMatrix(hessian);
            solver.data()->setGradient(gradient);
            solver.data()->setNumberOfConstraints(n_constraints);
        }
        // solver.data()->setLinearConstraintsMatrix(linearMatrix);
        // solver.data()->setLowerBound(lowerBound);
        // solver.data()->setUpperBound(upperBound);
        std::cout << "check_loop." << std::endl;
        // for every obstacle
        for (int i = 0; i < obstacles.size(); i++)
        {
            Eigen::MatrixXd O = obstacles[i].leftCols<3>();
            Eigen::VectorXd o = - obstacles[i].rightCols<1>();
            if (i == 0)
            {
                int row = 0;
                // (1) lambda >= 0
                for (int i = 0; i < n_lambda; ++i) 
                {
                    tripletList.emplace_back(row++, i, 1.0); // x[i] >= 0
                }
                // (2) mu >= 0
                for (int i = 0; i < n_mu; ++i) 
                {
                    tripletList.emplace_back(row++, n_lambda + i, 1.0); // x[i] >= 0
                }
                // (3) -o^T lambda - b^T mu >= d_min
                for (int i = 0; i < n_lambda; ++i) {
                    tripletList.emplace_back(row, i, -o(i));
                }
                for (int i = 0; i < n_mu; ++i) {
                    tripletList.emplace_back(row, n_lambda + i, -b(i));
                }
                row++;
                // (4) A^T *mu + O^T *lambda = 0
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < n_lambda; ++j) {
                        tripletList.emplace_back(row, j, O.coeff(j, i));
                    }
                    for (int j = 0; j < n_mu; ++j) {
                        tripletList.emplace_back(row, n_lambda + j, A.coeff(j, i));
                    }
                    row++;
                }
                // (5) (O* v_k)^T * lambda <= 1 , k = 1, ..., n_v
                for (int k = 0; k < n_v; ++k) {
                    Eigen::VectorXd w = O * V.row(k).transpose(); // n_lambda x 1
                    for (int i = 0; i < n_lambda; ++i) {
                        tripletList.emplace_back(row, i, w(i));
                    }
                    row++;
                }
                std::cout << "linearMatrix.setFromTriplets()." << std::endl;
                linearMatrix.setFromTriplets(tripletList.begin(), tripletList.end());
                // set the lower and upper bounds
                int idx = 0;
                // (1) lambda >= 0
                lowerBound.segment(idx, n_lambda).setZero();
                idx += n_lambda;
                // (2) mu >= 0
                lowerBound.segment(idx, n_mu).setZero();
                idx += n_mu;
                // (3) >= d_min
                lowerBound(idx) = d_min;
                idx++;
                // (4)  = 0
                lowerBound.segment(idx, 3).setZero();
                upperBound.segment(idx, 3).setZero();
                idx += 3;
                // (5) (O v_k)^T lambda <= 1
                upperBound.segment(idx, n_v).setOnes();
                // idx += n_v;
                solver.data()->clearLinearConstraintsMatrix();
                solver.data()->setLinearConstraintsMatrix(linearMatrix);
                solver.data()->setLowerBound(lowerBound);
                solver.data()->setUpperBound(upperBound);
                solver.initSolver();
                std::cout << "solver.initSolver()." << std::endl;
            }
            else
            {
                tripletList.clear();
                int row = n_lambda + n_mu;
                // (3) -o^T lambda - b^T mu >= d_min
                for (int i = 0; i < n_lambda; ++i) {
                    tripletList.emplace_back(row, i, -o(i));
                }
                for (int i = 0; i < n_mu; ++i) {
                    tripletList.emplace_back(row, n_lambda + i, -b(i));
                }
                row++;
                // (4) A^T *mu + O^T *lambda = 0
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < n_lambda; ++j) {
                        tripletList.emplace_back(row, j, O.coeff(j, i));
                    }
                    for (int j = 0; j < n_mu; ++j) {
                        tripletList.emplace_back(row, n_lambda + j, A.coeff(j, i));
                    }
                    row++;
                }
                // (5) (O* v_k)^T * lambda <= 1 , k = 1, ..., n_v
                for (int k = 0; k < n_v; ++k) {
                    Eigen::VectorXd w = O * V.row(k).transpose();
                    for (int i = 0; i < n_lambda; ++i) {
                        tripletList.emplace_back(row, i, w(i));
                    }
                    row++;
                }
                linearMatrix.setFromTriplets(tripletList.begin(), tripletList.end());
                solver.updateLinearConstraintsMatrix(linearMatrix);
                std::cout << "solver.updateLinearConstraintsMatrix()." << std::endl;
            }
            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            {
                return false;
            }
        }
        return true;
    }


    inline void scalePolytope(
        Eigen::MatrixX4d &meta_polytope,  // current meta polytope in H-rep format
        const double &d_min, 
        const Eigen::Vector3d &p_current, 
        const std::vector<Eigen::MatrixX4d> &obstacles,
        OsqpEigen::Solver &solver,
        const double tol = 1.0e-6
        )
        {
            Eigen::MatrixX3d A_c;
            Eigen::VectorXd b_c;
            Eigen::MatrixX3d Y_m = meta_polytope.leftCols<3>();
            Eigen::VectorXd y_m = - meta_polytope.rightCols<1>();
            double alpha_low = 0.0;
            double alpha_high = 1.0;
            double alpha_mid;
            while (alpha_high - alpha_low > tol)
            {
                alpha_mid = (alpha_low + alpha_high) / 2.0;
                std::cout << "alpha:" << alpha_mid << std::endl;
                A_c = Y_m;
                b_c = alpha_mid * y_m + Y_m * (1.0 - alpha_mid) * p_current;
                if (check_polytope_feasibility(A_c, b_c, obstacles, d_min, solver))
                {
                    alpha_low = alpha_mid;
                }
                else
                {
                    alpha_high = alpha_mid;
                }
            }
            // set new b
            std::cout << "alpha_opt:" << alpha_low << std::endl;
            meta_polytope.rightCols<1>() = -(alpha_low * y_m + Y_m * (1.0 - alpha_low) * p_current);
        }
} 

# endif