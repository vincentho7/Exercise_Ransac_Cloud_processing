#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <obj.h>

bool estimate_plane(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, 
                    Eigen::Vector3f &centroid, Eigen::Vector3f &normal);
float point_to_plane_distance(const Eigen::Vector3f &point, const Eigen::Vector3f &centroid, const Eigen::Vector3f &normal);
std::vector<Eigen::Vector3f> select_3_random_points(std::vector<Eigen::Vector3f> points, std::vector<size_t> remaining_idx);

void ransac_part1(const std::vector<Eigen::Vector3f> &points, int iterations, float distance_threshold, std::vector<Eigen::Vector3f> &colors);
std::vector<size_t> ransac(const std::vector<Eigen::Vector3f> &points, int iterations, float distance_threshold, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals, std::vector<size_t> remaining_idx, int colorIndex);
Eigen::Vector3f generate_random_color(std::mt19937 &rng);
void ransac_multiple_planes(std::vector<Eigen::Vector3f>& points, int iterations, float distance_threshold, std::vector<Eigen::Vector3f>& colors, std::vector<Eigen::Vector3f>& normals);
