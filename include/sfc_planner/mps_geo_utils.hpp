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

#ifndef MPS_GEO_UTILS_HPP
#define MPS_GEO_UTILS_HPP



#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

namespace mps_geo_utils
{
    inline std::vector<Eigen::MatrixX4d> loadObstaclesFromYAML(const std::string& filepath) 
    {
        std::vector<Eigen::MatrixX4d> hPolys;
            YAML::Node config = YAML::LoadFile(filepath);
            if (!config["hpolys"]) {
                return hPolys;
            }
            for (const auto& polytope_node : config["hpolys"]) {
                int num_constraints = polytope_node.size();
                Eigen::MatrixX4d H(num_constraints, 4);
                for (int i = 0; i < num_constraints; ++i) {
                    const auto& row = polytope_node[i];
                    H(i, 0) = row[0].as<double>();
                    H(i, 1) = row[1].as<double>();
                    H(i, 2) = row[2].as<double>();
                    H(i, 3) = row[3].as<double>(); 
                }
                hPolys.push_back(H);
            }
        return hPolys;
    }


} // namespace geo_utils

#endif
