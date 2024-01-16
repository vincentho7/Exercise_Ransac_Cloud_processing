#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <obj.h>
#include "ransac.hh"
#include <chrono>

int main(int argc, char const *argv[]) {
    if(argc <= 1) {
        std::cout << "Error: missing argument" << std::endl;
        std::cout << "Usage: ransac <filename>.obj" << std::endl;
        return 0;
    }
    const std::string filename = argv[1];

    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> normals;
    std::vector<Eigen::Vector3f> colors;
    
    if(not tnp::load_obj(filename, points, normals, colors)) {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    if (colors.size() == 0){
        colors = std::vector<Eigen::Vector3f>(points.size(), Eigen::Vector3f(0.5f, 0.5f, 0.5f));
    }
    // RANSAC parameters
    int iterations = 100; // Number of iterations
    float distance_threshold = 0.5f; // Distance threshold for inliers
    // Run RANSAC
    auto start = std::chrono::high_resolution_clock::now();
    ransac_multiple_planes(points, iterations, distance_threshold, colors, normals);
    auto end = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double> duration = end - start;
    std::cout << "RANSAC took " << duration.count() << " seconds." << std::endl;
    // Your code to handle the results...
    tnp::save_obj("mult_plan.obj", points, normals, colors);

    return 0;
}