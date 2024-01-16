#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <obj.h>

// Function to estimate a plane from three points
void estimate_plane(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, 
                    Eigen::Vector3f &centroid, Eigen::Vector3f &normal) {
    // Compute the centroid and normal of the plane
    centroid = (p1 + p2 + p3) / 3;
    Eigen::Vector3f u = p2 - p1;
    Eigen::Vector3f v = p3 - p1;
    normal = u.cross(v).normalized();
}

// Function to calculate the distance from a point to a plane
float point_to_plane_distance(const Eigen::Vector3f &point, const Eigen::Vector3f &centroid, const Eigen::Vector3f &normal) {
    return std::abs(normal.dot(point - centroid));
}

std::vector<Eigen::Vector3f> select_3_random_points(std::vector<Eigen::Vector3f> points, std::vector<size_t> remaining_idx){
    std::vector<Eigen::Vector3f> random_points;
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<> dist(0, remaining_idx.size()-1);
    
    for(int i = 0; i < 3; ++i) {
        Eigen::Vector3f p = points[remaining_idx[dist(rng)]];
        if (std::find(random_points.begin(), random_points.end(), p) == random_points.end()){
            random_points.push_back(p);
        }
    }
    return random_points;
}

Eigen::Vector3f generate_random_color() {
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return Eigen::Vector3f(dist(rng), dist(rng), dist(rng));
}

// Main RANSAC algorithm to find the best plane
std::vector<size_t> ransac(const std::vector<Eigen::Vector3f> &points, int iterations, float distance_threshold, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals, std::vector<size_t> remaining_idx) {
    int best_inlier_count = 0;
    Eigen::Vector3f best_p;
    Eigen::Vector3f best_n;

    for(int i = 0; i < iterations; ++i) {
        // Randomly select 3 different points
        std::vector<Eigen::Vector3f> sample_points = select_3_random_points(points, remaining_idx);
        
        Eigen::Vector3f centroid, normal;
        estimate_plane(sample_points[0], sample_points[1], sample_points[2], centroid, normal);

        // Count inliers
        int inlier_count = 0;
        for(const auto &point : points) {
            if(point_to_plane_distance(point, centroid, normal) < distance_threshold) {
                ++inlier_count;
            }
        }

        // Update best plane if current one has more inliers
        if(inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_p = centroid;
            best_n = normal;
        }
    }

    std::vector<size_t> new_remaining_idx;
    for(size_t i = 0; i < remaining_idx.size(); i++) {
        Eigen::Vector3f point = points[remaining_idx[i]];
        if(point_to_plane_distance(point, best_p, best_n) < distance_threshold) {
            colors[remaining_idx[i]] = generate_random_color();
        }
        else {
            new_remaining_idx.push_back(remaining_idx[i]);
        }
    }
    return new_remaining_idx;
}

void ransac_multiple_planes(std::vector<Eigen::Vector3f>& points, int iterations, float distance_threshold, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals){
    std::vector<size_t> remaining_idx;
    for (size_t i = 0; i < points.size(); ++i) {
        remaining_idx.push_back(i);
    }

    while (static_cast<float>(remaining_idx.size()) / static_cast<float>(points.size()) > 0.25f)
    {
        remaining_idx = ransac(points, iterations, distance_threshold, colors, normals, remaining_idx);
    }
}


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
    float distance_threshold = 0.1f; // Distance threshold for inliers
    // Run RANSAC
    // ransac(points, iterations, distance_threshold, colors);
    ransac_multiple_planes(points, iterations, distance_threshold, colors, normals);

    // Your code to handle the results...
    tnp::save_obj("result.obj", points, normals, colors);

    return 0;
}
