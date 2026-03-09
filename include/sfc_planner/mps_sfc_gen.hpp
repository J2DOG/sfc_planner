/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef MPS_SFC_GEN_HPP
#define MPS_SFC_GEN_HPP

#include "mps.hpp"

#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/PathSimplifier.h>

#include <deque>
#include <ros/ros.h>
#include <memory>
#include <Eigen/Eigen>
#include <chrono>

#include "OsqpEigen/OsqpEigen.h"

namespace mps_sfc_gen
{

    template <typename Map>
    // plan a path(p) by informed RRT*
    inline double planPath(const Eigen::Vector3d &s,
                            const Eigen::Vector3d &g,
                            const Eigen::Vector3d &lb,
                            const Eigen::Vector3d &hb,
                            const Map *mapPtr,
                            const double &timeout,
                            std::vector<Eigen::Vector3d> &p)
    {
        // setup the state space
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
        // setup the bounds with zero origin
        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));
        space->setBounds(bounds);
        // setup the space information
        auto si(std::make_shared<ompl::base::SpaceInformation>(space));

        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                                lb(1) + (*pos)[1],
                                                lb(2) + (*pos)[2]);
                return mapPtr->query(position) == 0;
            });
        si->setup();

        ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);
        start[1] = s(1) - lb(1);
        start[2] = s(2) - lb(2);
        goal[0] = g(0) - lb(0);
        goal[1] = g(1) - lb(1);
        goal[2] = g(2) - lb(2);

        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
        auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
        planner->setProblemDefinition(pdef);
        planner->setup();
        // set the max range of each step
        planner->setRange(0.05 * (hb - lb).norm());

        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

        double cost = INFINITY;
        if (solved)
        {
            p.clear();
            // Simplify
            ompl::geometric::PathGeometric path_(
                dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            ompl::geometric::PathSimplifier Simplifier(si);
            Simplifier.simplify(path_, 1.0);
            for (size_t i = 0; i < path_.getStateCount(); ++i)
                {
                    const auto* state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
                    p.emplace_back(state->values[0] + lb(0), state->values[1] + lb(1), state->values[2] + lb(2));
                }
            cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        }
        return cost;
    }

    // convexCover: Generate a sequence of convex polytopes (half-space form)
    // covering a piecewise-linear path.
    // Inputs:
    //  - path: ordered waypoints to cover
    //  - obstacles: obstacles in H-rep format
    //  - lowCorner/highCorner: map bounds (min/max corners of the workspace)
    //  - Meta_poly: meta polytope in H-rep format around zero point [A -b]
    // Output:
    //  - hpolys: vector of half-space matrices (rows: [nx ny nz d]) defining each polytope
    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::MatrixX4d> &obstacles,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const Eigen::MatrixX4d &Meta_poly,
                            const double &d_min,
                            std::vector<Eigen::MatrixX4d> &hpolys
                            )
    {
        OsqpEigen::Solver solver;
        //Init solver
        int n_constraints = 6 + 6 + 1 + 3 + 36;
        int dim = 12;
        Eigen::SparseMatrix<double> linearMatrix(n_constraints, dim);
        Eigen::VectorXd lowerBound = Eigen::VectorXd::Constant(n_constraints, -1.0e9);
        Eigen::VectorXd upperBound = Eigen::VectorXd::Constant(n_constraints, 1.0e9);
        Eigen::SparseMatrix<double> hessian(dim, dim);
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dim);
        solver.data()->setNumberOfVariables(dim);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(gradient);
        solver.data()->setNumberOfConstraints(n_constraints);
        solver.data()->setBounds(lowerBound, upperBound);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        // setting
        solver.settings()->setVerbosity(false);
        solver.initSolver();
        hpolys.clear();
        const int n = path.size();
        // set current position
        Eigen::Vector3d p_current = path[0];
        int path_idx = 0;
        int n_path = path.size();
        // generate 6 walls bounding the map. push back to the valid_obstacles. thick 0.01m.
        std::vector<Eigen::MatrixX4d> valid_obstacles = obstacles;
        Eigen::MatrixX4d wall_0 = Eigen::MatrixX4d::Zero(6, 4);
        wall_0(0, 0) = 1.0;
        wall_0(1, 0) = -1.0;
        wall_0(2, 1) = 1.0;
        wall_0(3, 1) = -1.0;
        wall_0(4, 2) = 1.0;
        wall_0(5, 2) = -1.0;
        wall_0(0, 3) = -lowCorner(0);
        wall_0(1, 3) = lowCorner(0)-0.01;
        wall_0(2, 3) = -highCorner(1);
        wall_0(3, 3) = lowCorner(1);
        wall_0(4, 3) = -highCorner(2);
        wall_0(5, 3) = highCorner(2);
        valid_obstacles.push_back(wall_0);
        Eigen::MatrixX4d wall_1 = Eigen::MatrixX4d::Zero(6, 4);
        wall_1(0, 0) = 1.0;
        wall_1(1, 0) = -1.0;
        wall_1(2, 1) = 1.0;
        wall_1(3, 1) = -1.0;
        wall_1(4, 2) = 1.0;
        wall_1(5, 2) = -1.0;
        wall_1(0, 3) = -highCorner(0)-0.01;
        wall_1(1, 3) = highCorner(0);
        wall_1(2, 3) = -highCorner(1);
        wall_1(3, 3) = lowCorner(1);
        wall_1(4, 3) = -highCorner(2);
        wall_1(5, 3) = highCorner(2);
        valid_obstacles.push_back(wall_1);
        Eigen::MatrixX4d wall_2 = Eigen::MatrixX4d::Zero(6, 4);
        wall_2(0, 0) = 1.0;
        wall_2(1, 0) = -1.0;
        wall_2(2, 1) = 1.0;
        wall_2(3, 1) = -1.0;
        wall_2(4, 2) = 1.0;
        wall_2(5, 2) = -1.0;
        wall_2(0, 3) = -highCorner(0);
        wall_2(1, 3) = lowCorner(0);
        wall_2(2, 3) = -lowCorner(1);
        wall_2(3, 3) = lowCorner(1)-0.01;
        wall_2(4, 3) = -highCorner(2);
        wall_2(5, 3) = highCorner(2);
        valid_obstacles.push_back(wall_2);
        Eigen::MatrixX4d wall_3 = Eigen::MatrixX4d::Zero(6, 4);
        wall_3(0, 0) = 1.0;
        wall_3(1, 0) = -1.0;
        wall_3(2, 1) = 1.0;
        wall_3(3, 1) = -1.0;
        wall_3(4, 2) = 1.0;
        wall_3(5, 2) = -1.0;
        wall_3(0, 3) = -highCorner(0);
        wall_3(1, 3) = lowCorner(0);
        wall_3(2, 3) = -highCorner(1)-0.01;
        wall_3(3, 3) = highCorner(1);
        wall_3(4, 3) = -highCorner(2);
        wall_3(5, 3) = highCorner(2);
        valid_obstacles.push_back(wall_3);
        Eigen::MatrixX4d wall_4 = Eigen::MatrixX4d::Zero(6, 4);
        wall_4(0, 0) = 1.0;
        wall_4(1, 0) = -1.0;
        wall_4(2, 1) = 1.0;
        wall_4(3, 1) = -1.0;
        wall_4(4, 2) = 1.0;
        wall_4(5, 2) = -1.0;
        wall_4(0, 3) = -highCorner(0);
        wall_4(1, 3) = lowCorner(0);
        wall_4(2, 3) = -highCorner(1);
        wall_4(3, 3) = lowCorner(1);
        wall_4(4, 3) = -lowCorner(2);
        wall_4(5, 3) = lowCorner(2)-0.01;
        valid_obstacles.push_back(wall_4);
        Eigen::MatrixX4d wall_5 = Eigen::MatrixX4d::Zero(6, 4);
        wall_5(0, 0) = 1.0;
        wall_5(1, 0) = -1.0;
        wall_5(2, 1) = 1.0;
        wall_5(3, 1) = -1.0;
        wall_5(4, 2) = 1.0;
        wall_5(5, 2) = -1.0;
        wall_5(0, 3) = -highCorner(0);
        wall_5(1, 3) = lowCorner(0);
        wall_5(2, 3) = -highCorner(1);
        wall_5(3, 3) = lowCorner(1);
        wall_5(4, 3) = -highCorner(2)-0.01;
        wall_5(5, 3) = highCorner(2);
        valid_obstacles.push_back(wall_5);
        //Meta polytope around current position [A -b-A*p_current]
        const int m_meta = Meta_poly.rows();
        Eigen::MatrixX3d A_meta = Meta_poly.leftCols<3>();
        Eigen::VectorXd b_meta = -Meta_poly.rightCols<1>();
        Eigen::MatrixX4d poly_current = Eigen::MatrixX4d::Zero(m_meta, 4);
        Eigen::MatrixX3d A_poly;
        Eigen::VectorXd b_poly;
        Eigen::Vector3d p_poly;
        // set input polytope
        Eigen::Vector3d x_positive(1.0, 0.0, 0.0);
        Eigen::Vector3d target_direction;
        if (path.size() > 1)
        {
            target_direction = 1.0*path[1] - p_current;
            //target_direction[2] = 0;
            target_direction = target_direction.normalized();
        }
        else
        {
            target_direction = path.back() - p_current;
            //target_direction[2] = 0;
            target_direction = target_direction.normalized();
        }
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(x_positive, target_direction);
        Eigen::Matrix3d R = q.toRotationMatrix();
        A_poly = A_meta * R.transpose();
        b_poly = b_meta + A_poly * p_current;
        poly_current.leftCols<3>() = A_poly;
        poly_current.rightCols<1>() = - b_poly;
        // scale the first polytope
        mps::scalePolytope(poly_current, d_min, p_current, valid_obstacles);
        A_poly = poly_current.leftCols<3>();
        b_poly = -poly_current.rightCols<1>();
        p_poly = path[0];
        ROS_INFO("scalePolytope 0");
        hpolys.push_back(poly_current);
        int next_idx = 0;
        double len_high = 1.0;
        double len_low = 0.0;
        double len_mid;
        path_idx = 0   ; // start from second point
        // generate the rest of the polytopes
        auto start = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(10);
        while(true)
        {
            auto now = std::chrono::steady_clock::now();
            if (now - start > timeout or hpolys.size() > 10)
            {
                break; 
            }
            // find the biggest path_idx inside the polytope
            for (int i = path_idx; i < n_path; i++)
            {
                if ((A_poly * path[i] - b_poly).maxCoeff() < 0.0) 
                {
                    next_idx = i;
                    if (next_idx == n_path - 1)
                    {
                        ROS_INFO("[sfc_gen]: Goal is inside the polytope. Return SFC.");
                        return;
                    }
                }
            }
            
            // if no points in the cube insert one
            if (next_idx > path_idx)
            {
                path_idx = next_idx;
                std::cout<<"Process path_idx " << path_idx << " in " << path.size() << std::endl;
                p_current = path[path_idx];
                target_direction = path[path_idx + 1] - p_current;
            }
            else
            {
                // now: path_idx == next_idx;
                // path_idx ++;
                // if (path_idx > n_path - 1)
                // {
                //     break;
                // }
                // std::cout<<" one point escape" << std::endl;
                // p_current = path[path_idx];
                // target_direction = path[path_idx + 1] - p_current;
                // now path_idx+1 is outside the currenct polytope, p_poly is inside
                std::cout<<"path_idx:"<< path_idx << " needs some help." << std::endl;
                if ((A_poly * p_poly - b_poly).maxCoeff() < 0.0) 
                {
                    std::cout<<"p_poly: inside"<< std::endl;
                }
                if ((A_poly * path[path_idx+1] - b_poly).maxCoeff() >= 0.0) 
                {
                    std::cout<<"path[path_idx+1]: outside"<< std::endl;
                }
                int it = 0;
                len_high = 1.0;
                len_low = 0.0;
                Eigen::Vector3d p_next;
                while (len_high - len_low > 1.0e-2)
                {
                    it++;
                    if (it > 500)
                    {
                        std::cout<<"insert one point beyond max iteration." << std::endl;
                        break;
                    }
                    
                    len_mid = (len_high+len_low)/2.0;
                    
                    p_next = p_poly + len_mid*(path[path_idx+1]- p_poly);
                    
                    
                    if ((A_poly * p_next - b_poly).maxCoeff() < 0.0) 
                    {
                        // std::cout<<"inside: len_low = len_mid = " << len_mid << std::endl;
                        len_low = len_mid;
                    }
                    else
                    {
                        len_high = len_mid;
                    }
                }
                p_current = p_poly + len_low*(path[path_idx+1]- p_poly);
                std::cout << "p_poly: " 
                            << p_poly[0] << " " 
                            << p_poly[1] << " " 
                            << p_poly[2] << std::endl;
                std::cout << "path[path_idx+1]: " 
                            << path[path_idx+1][0] << " " 
                            << path[path_idx+1][1] << " " 
                            << path[path_idx+1][2] << ", len_low:"<< len_low << std::endl;
            }
            
            if (path_idx == n_path - 1)
            {
                target_direction = path[path_idx] - path[path_idx-1];
            }
            //target_direction[2] = 0;
            target_direction = target_direction.normalized();
            q = Eigen::Quaterniond::FromTwoVectors(x_positive, target_direction);
            R = q.toRotationMatrix();
            A_poly = A_meta * R.transpose();
            b_poly = b_meta + A_poly * p_current;
            p_poly = p_current; // update p_poly
            poly_current.leftCols<3>() = A_poly;
            poly_current.rightCols<1>() = - b_poly;
            mps::scalePolytope(poly_current, d_min, p_current, valid_obstacles);
            A_poly = poly_current.leftCols<3>();
            b_poly = -poly_current.rightCols<1>();
            hpolys.push_back(poly_current);
        }
        ROS_WARN("[sfc_gen]: Cant reach the goal.");

    }
}

#endif
