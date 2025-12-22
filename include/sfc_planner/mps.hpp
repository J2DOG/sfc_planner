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
        int n_v = 36;
        int n_constraints = 0;
        Eigen::MatrixXd V_A(n_v, 3);
        Eigen::MatrixXd V_b(n_v, 1);
        V_A <<
            0.736540492226054,	0.102910070414198,	-0.127374228765269,
            0.618164183289846,	-0.0196711720961478,	-0.433909224956971,
            0.465513636647270,	0.165972011761052,	0.574389689641919,
            0.579103136560661,	0.503082007140221,	0.0641508379483852,
            0.448369178401130,	0.632045826688227,	0.00110715486109160,
            -0.0670301379108451,	-0.0694354299903613	-0.750349237302986,
            -0.361630145552854,	-0.181864467401454,	-0.635745630313536,
            -0.0196048475753641,	-0.632045826688227,	0.447941732688012,
            0.0373327524701596,	-0.503082007140221,	0.581448225018253,
            0.552273841262034,	-0.165972011761052,	0.491548640247518,
            0.669651875417235,	-0.258539967489045,	0.228425722686292,
            -0.209997810258490,	-0.651245741367350,	0.328114139269282,
            -0.236411259240224,	-0.651487916378421,	-0.308604449304055,
            -0.395593580054606,	-0.528905624968057,	-0.374714929400359,
            -0.708808111015699,	-0.272155202601645,	0.0269680128651459,
            -0.627632966256385,	-0.336100737607103,	0.249078002128006,
            0.0709294549360598,	0.670108779820913,	0.311251439894383,
            0.444289078246690,	-0.393588982363609,	-0.481860529700963,
            0.321409506589788,	-0.421575131045455,	-0.544084366539299,
            -0.108995524351576,	-0.662782412822596,	-0.330996861280345,
            0.307642877734650,	-0.670108779820913,	0.0852308992853917,
            0.499424306022766,	-0.560528827226044,	-0.00305615489318134,
            0.337464014540313,	0.651245741367350,	-0.194617593223526,
            0.277803411868556,	0.336100737607103,	-0.615457761207101,
            0.0596800988984874,	0.272155202601645,	-0.706805841624844,
            0.197249713923002,	0.258539967489045,	0.679488407106308,
            -0.0261220236031970, 0.560528827226044,	0.498750055049547,
            -0.501868549359329,	0.393588982363609,	0.421557011917663,
            -0.461999970089482,	0.0196711720961483,	0.597461463730072,
            -0.161260140543013,	-0.102910070414198,	0.729870713150071,
            -0.356041921276499,	0.528905624968058,	-0.412479950045665,
            -0.558349983724806,	0.421575131045455,	0.295934392244750,
            -0.325608896178765,	0.662782412822596,	-0.124168406752123,
            -0.297354868709902,	0.651487916378421,	-0.250413800898283,
            -0.746452099658915,	0.0694354299903611,	-0.101618306541274,
            -0.618362819602740,	0.181864467401455,	-0.390610153437262;
        V_b <<
            0.656272372239758,
            0.655140345256802,
            0.652554081927685,
            0.638304567826278,
            0.632045826688227,
            0.653958487789963,
            0.657249151282413,
            0.632045826688227,
            0.638304567826277,
            0.652554081927685,
            0.657681640443299,
            0.651245741367350,
            0.651487916378421,
            0.650656038918577,
            0.650227140111300,
            0.656561728022654,
            0.670108779820914,
            0.644597050744876,
            0.650340326325568,
            0.662782412822596,
            0.670108779820913,
            0.660585691882244,
            0.651245741367350,
            0.656561728022654,
            0.650227140111300,
            0.657681640443300,
            0.660585691882244,
            0.644597050744876,
            0.655140345256802,
            0.656272372239758,
            0.650656038918577,
            0.650340326325568,
            0.662782412822597,
            0.651487916378421,
            0.653958487789963,
            0.657249151282413;
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
                // (5) V_A * O^T * lambda <= V_b , k = 1, ..., n_v
                Eigen::MatrixXd V_A_O = V_A * O.transpose();
                for (int k = 0; k < n_v; ++k) {
                    for (int i = 0; i < n_lambda; ++i) {
                        tripletList.emplace_back(row, i, V_A_O(k, i));
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
                // (5) V_A * O^T * lambda <= V_b , k = 1, ..., n_v
                upperBound.segment(idx, n_v) = V_b;
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
                // (5) V_A * O^T * lambda <= V_b , k = 1, ..., n_v
                Eigen::MatrixXd V_A_O = V_A * O.transpose();
                for (int k = 0; k < n_v; ++k) {
                    for (int i = 0; i < n_lambda; ++i) {
                        tripletList.emplace_back(row, i, V_A_O(k, i));
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