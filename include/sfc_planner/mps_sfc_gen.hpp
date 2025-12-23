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

#include <deque>
#include <ros/ros.h>
#include <memory>
#include <Eigen/Eigen>

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
            const ompl::geometric::PathGeometric path_ =
                ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            for (size_t i = 0; i < path_.getStateCount(); i++)
            {
                const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
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
        // Eigen::MatrixX4d wall_0 = Eigen::MatrixX4d::Zero(6, 4);
        // wall_0(0, 0) = 1.0;
        // wall_0(1, 0) = -1.0;
        // wall_0(2, 1) = 1.0;
        // wall_0(3, 1) = -1.0;
        // wall_0(4, 2) = 1.0;
        // wall_0(5, 2) = -1.0;
        // wall_0(0, 3) = -lowCorner(0);
        // wall_0(1, 3) = lowCorner(0)-0.01;
        // wall_0(2, 3) = -highCorner(1);
        // wall_0(3, 3) = lowCorner(1);
        // wall_0(4, 3) = -highCorner(2);
        // wall_0(5, 3) = highCorner(2);
        // valid_obstacles.push_back(wall_0);
        // Eigen::MatrixX4d wall_1 = Eigen::MatrixX4d::Zero(6, 4);
        // wall_1(0, 0) = 1.0;
        // wall_1(1, 0) = -1.0;
        // wall_1(2, 1) = 1.0;
        // wall_1(3, 1) = -1.0;
        // wall_1(4, 2) = 1.0;
        // wall_1(5, 2) = -1.0;
        // wall_1(0, 3) = -highCorner(0)-0.01;
        // wall_1(1, 3) = highCorner(0);
        // wall_1(2, 3) = -highCorner(1);
        // wall_1(3, 3) = lowCorner(1);
        // wall_1(4, 3) = -highCorner(2);
        // wall_1(5, 3) = highCorner(2);
        // valid_obstacles.push_back(wall_1);
        // Eigen::MatrixX4d wall_2 = Eigen::MatrixX4d::Zero(6, 4);
        // wall_2(0, 0) = 1.0;
        // wall_2(1, 0) = -1.0;
        // wall_2(2, 1) = 1.0;
        // wall_2(3, 1) = -1.0;
        // wall_2(4, 2) = 1.0;
        // wall_2(5, 2) = -1.0;
        // wall_2(0, 3) = -highCorner(0);
        // wall_2(1, 3) = lowCorner(0);
        // wall_2(2, 3) = -lowCorner(1);
        // wall_2(3, 3) = lowCorner(1)-0.01;
        // wall_2(4, 3) = -highCorner(2);
        // wall_2(5, 3) = highCorner(2);
        // valid_obstacles.push_back(wall_2);
        // Eigen::MatrixX4d wall_3 = Eigen::MatrixX4d::Zero(6, 4);
        // wall_3(0, 0) = 1.0;
        // wall_3(1, 0) = -1.0;
        // wall_3(2, 1) = 1.0;
        // wall_3(3, 1) = -1.0;
        // wall_3(4, 2) = 1.0;
        // wall_3(5, 2) = -1.0;
        // wall_3(0, 3) = -highCorner(0);
        // wall_3(1, 3) = lowCorner(0);
        // wall_3(2, 3) = -highCorner(1)-0.01;
        // wall_3(3, 3) = highCorner(1);
        // wall_3(4, 3) = -highCorner(2);
        // wall_3(5, 3) = highCorner(2);
        // valid_obstacles.push_back(wall_3);
        // Eigen::MatrixX4d wall_4 = Eigen::MatrixX4d::Zero(6, 4);
        // wall_4(0, 0) = 1.0;
        // wall_4(1, 0) = -1.0;
        // wall_4(2, 1) = 1.0;
        // wall_4(3, 1) = -1.0;
        // wall_4(4, 2) = 1.0;
        // wall_4(5, 2) = -1.0;
        // wall_4(0, 3) = -highCorner(0);
        // wall_4(1, 3) = lowCorner(0);
        // wall_4(2, 3) = -highCorner(1);
        // wall_4(3, 3) = lowCorner(1);
        // wall_4(4, 3) = -lowCorner(2);
        // wall_4(5, 3) = lowCorner(2)-0.01;
        // valid_obstacles.push_back(wall_4);
        // Eigen::MatrixX4d wall_5 = Eigen::MatrixX4d::Zero(6, 4);
        // wall_5(0, 0) = 1.0;
        // wall_5(1, 0) = -1.0;
        // wall_5(2, 1) = 1.0;
        // wall_5(3, 1) = -1.0;
        // wall_5(4, 2) = 1.0;
        // wall_5(5, 2) = -1.0;
        // wall_5(0, 3) = -highCorner(0);
        // wall_5(1, 3) = lowCorner(0);
        // wall_5(2, 3) = -highCorner(1);
        // wall_5(3, 3) = lowCorner(1);
        // wall_5(4, 3) = -highCorner(2)-0.01;
        // wall_5(5, 3) = highCorner(2);
        // valid_obstacles.push_back(wall_5);
        // Meta polytope around current position [A -b-A*p_current]
        const int m_meta = Meta_poly.rows();
        Eigen::MatrixX4d poly_current = Eigen::MatrixX4d::Zero(m_meta, 4);
        poly_current.leftCols<3>() = Meta_poly.leftCols<3>();
        poly_current.rightCols<1>() = Meta_poly.rightCols<1>() - Meta_poly.leftCols<3>() * p_current;
        // scale the first polytope
        ROS_INFO("scalePolytope()");
        mps::scalePolytope(poly_current, d_min, p_current, valid_obstacles, solver);
        Eigen::MatrixX3d A_poly = poly_current.leftCols<3>();
        Eigen::VectorXd b_poly = -poly_current.rightCols<1>();
        hpolys.push_back(poly_current);
        // generate the rest of the polytopes
        for (path_idx = 1; path_idx < n_path; path_idx++)
        {
            // find the biggest path_idx inside the polytope
            for (int i = path_idx; i < n_path; i++)
            {
                if ((A_poly * path[i] - b_poly).maxCoeff() < 0.0) 
                {
                    path_idx = i;
                    if (path_idx == n_path - 1)
                    {
                        ROS_INFO("[sfc_gen]: Goal is inside the polytope. Return SFC.");
                        return;
                    }
                }
            }
            std::cout<<"path_idx" << path_idx << "in" << path.size() << std::endl;
            p_current = path[path_idx];
            poly_current.leftCols<3>() = Meta_poly.leftCols<3>();
            poly_current.rightCols<1>() = Meta_poly.rightCols<1>() - Meta_poly.leftCols<3>() * p_current;
            mps::scalePolytope(poly_current, d_min, p_current, valid_obstacles, solver);
            A_poly = poly_current.leftCols<3>();
            b_poly = -poly_current.rightCols<1>();
            hpolys.push_back(poly_current);
        }
    }

}

#endif
