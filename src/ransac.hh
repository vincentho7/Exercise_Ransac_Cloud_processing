#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <obj.h>

namespace RANSAC{
    // RANSAC parameters
    extern int iterations; // Number of iterations
    extern float dist_threshold; // Distance threshold for inliers
    extern float align_threshold;
    extern float pointsleft;

    void estimate_plane(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, 
                        Eigen::Vector3f &centroid, Eigen::Vector3f &normal);
    float point_to_plane_distance(const Eigen::Vector3f &point, const Eigen::Vector3f &centroid, const Eigen::Vector3f &normal);
    std::vector<Eigen::Vector3f> select_3_random_points(std::vector<Eigen::Vector3f> points, std::vector<size_t> remaining_idx);

    void simple_ransac(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &colors);
    
    std::vector<size_t> ransac(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals, std::vector<size_t> remaining_idx, int colorIndex);
    void ransac_multiple_planes(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals);
    
    std::vector<size_t> ransac_with_normals(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals, std::vector<size_t> remaining_idx, int colorIndex);
    void ransac_n_mult_planes(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals);
}