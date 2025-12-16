# ifndef MPS_HPP
# define MPS_HPP

# include <Eigen/Eigen>
# include <vector>

namespace mps
{

    inline void scalePolytope(
        Eigen::MatrixX4d &meta_polytope,  // current meta polytope in H-rep format
        const double &d_min, 
        const Eigen::Vector3d &p_current, 
        const std::vector<Eigen::MatrixX4d> &obstacles,
        const double tol = 1.0e-6)
        {
            Eigen::MatrixX3d A_c;
            Eigen::VectorXd b_c;
            Eigen::MatrixX3d Y_m = meta_polytope.leftCols<3>();
            Eigen::VectorXd y_m = -meta_polytope.rightCols<1>();
            double alpha_low = 0.0;
            double alpha_high = 1.0;
            double alpha_mid;
            while (alpha_high - alpha_low > tol)
            {
                alpha_mid = (alpha_low + alpha_high) / 2.0;
                A_c = Y_m;
                b_c = alpha_mid * y_m + Y_m * (1.0 - alpha_mid) * p_current;
                if (check_polytope_feasibility(A_c, b_c, obstacles, d_min))
                {
                    alpha_low = alpha_mid;
                }
                else
                {
                    alpha_high = alpha_mid;
                }
            }
            meta_polytope.rightCols<1>() = alpha_low * y_m + Y_m * (1.0 - alpha_low) * p_current;
        }
    // Check if the polytope is feasible for the given obstacles and minimum distance requirement
    inline bool check_polytope_feasibility(
        const Eigen::MatrixX4d &polytope, // polytope in H-rep (Ax<=b) format: [A -b]
        const std::vector<Eigen::MatrixX4d> &obstacles,
        const double &d_min) // minimum distance requirement
    {
        // Extract set H-rep: A x <= b
        Eigen::MatrixX3d A = polytope.leftCols<3>();
        Eigen::VectorXd b = -polytope.col(3);
        int max_iter = 50;
        double tol = 1.0e-6;
        const int m_p = A.rows();
        const int dim = 3;
        // Precompute AAt and its inverse if possible (shared across obstacles)
        Eigen::MatrixXd AAt = A * A.transpose();
        Eigen::FullPivLU<Eigen::MatrixXd> lu_AAt(AAt);
        bool AAt_invertible = lu_AAt.isInvertible();
        Eigen::MatrixXd AAt_inv;
        if (AAt_invertible) {
            AAt_inv = lu_AAt.inverse();
        }
        else {
            std::cout << "AAt is not invertible" << std::endl;
            return false;
        }
        for (const auto &obs : obstacles) {
            Eigen::MatrixX3d O = obs.leftCols(3);
            Eigen::VectorXd o = -obs.col(3);
            const int m_o = O.rows();
            // --- Solve dual feasibility for this obstacle ---
            // mu = M * lambda, where M = -(A A^T)^{-1} A O^T
            Eigen::MatrixXd M = -AAt_inv * A * O.transpose(); // (m_p x m_o)
            Eigen::VectorXd c = -o - b.transpose() * M;       // (m_o), want c^T lambda >= d_min
            Eigen::VectorXd lambda = Eigen::VectorXd::Zero(m_o);
            bool feasible = false;
            for (int iter = 0; iter < max_iter; ++iter) {
                double obj = c.dot(lambda);
                Eigen::VectorXd mu = M * lambda;
                double norm_z = (O.transpose() * lambda).norm();
                // Check all dual feasibility conditions
                if (obj >= d_min - tol &&
                    (mu.array() >= -tol).all() &&
                    (lambda.array() >= -tol).all() &&
                    norm_z <= 1.0 + tol) {
                    feasible = true;
                    break;
                }
    
                // Gradient ascent on c^T lambda
                double step = 0.2 / std::sqrt(iter + 1);
                lambda += step * c;
                lambda = lambda.cwiseMax(0.0); // lambda >= 0
    
                // Enforce mu >= 0 approximately
                mu = M * lambda;
                for (int j = 0; j < m_p; ++j) {
                    if (mu(j) < 0) {
                        // Project back by reducing components that cause violation
                        lambda -= 0.05 * mu(j) * M.row(j).transpose();
                        lambda = lambda.cwiseMax(0.0);
                    }
                }
    
                // Project onto ||O^T lambda||_2 <= 1
                double nz = (O.transpose() * lambda).norm();
                if (nz > 1.0) {
                    lambda *= (1.0 / nz);
                }
            } // end for iter
            if (!feasible) {
                return false; // Collision or too close to this obstacle
            }
        } // end for all obstacles
        return true; // Safe from all obstacles
    }
}

# endif