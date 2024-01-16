#pragma once
#include <Eigen/Core>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <random>

extern std::vector<Eigen::Vector3f> distinctColors;

Eigen::Vector3f generate_random_color();

Eigen::Vector3f generate_color(int colorIndex);