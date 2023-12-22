#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <obj.h> // Make sure this is the correct path to your header

// Function to estimate a plane from three points
bool estimate_plane(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, 
                    Eigen::Vector3f &centroid, Eigen::Vector3f &normal) {
    // Compute the centroid and normal of the plane
    centroid = (p1 + p2 + p3) / 3;
    Eigen::Vector3f u = p2 - p1;
    Eigen::Vector3f v = p3 - p1;
    normal = u.cross(v).normalized();
    if(normal.norm() == 0) return false; // The points are collinear
    return true;
}

// Function to calculate the distance from a point to a plane
float point_to_plane_distance(const Eigen::Vector3f &point, const Eigen::Vector3f &centroid, const Eigen::Vector3f &normal) {
    return std::abs(normal.dot(point - centroid));
}

// Main RANSAC algorithm to find the best plane
Eigen::Vector3f ransac(const std::vector<Eigen::Vector3f> &points, int iterations, float distance_threshold) {
    std::random_device rd;
    // Initialize a Mersenne Twister pseudo-random generator with a random seed from std::random_device
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist(0, points.size() - 1);

    int best_inlier_count = 0;
    Eigen::Vector3f best_centroid;
    Eigen::Vector3f best_normal;

    for(int i = 0; i < iterations; ++i) {
        // Randomly select 3 different points
        int idx1 = dist(rng), idx2 = dist(rng), idx3 = dist(rng);
        Eigen::Vector3f centroid, normal;
        if(!estimate_plane(points[idx1], points[idx2], points[idx3], centroid, normal)) continue;

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
            best_centroid = centroid;
            best_normal = normal;
        }
    }

    if(best_inlier_count > 0) {
        std::cout << "Best plane found with centroid: " << best_centroid.transpose() 
                  << " and normal: " << best_normal.transpose() << std::endl;
        std::cout << "Inliers count: " << best_inlier_count << std::endl;
    } else {
        std::cout << "No plane could be found." << std::endl;
    }

    return best_centroid; // or return whatever you need
}

int main(int argc, char const *argv[]) {
    if(argc <= 1) {
        std::cout << "Error: missing argument" << std::endl;
        std::cout << "Usage: ransac <filename>.obj" << std::endl;
        return 0;
    }
    const std::string filename = argv[1];

    std::vector<Eigen::Vector3f> points;
    
    if(not tnp::load_obj(filename, points)) {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    // RANSAC parameters
    int iterations = 1000; // Number of iterations
    float distance_threshold = 0.01f; // Distance threshold for inliers

    // Run RANSAC
    Eigen::Vector3f best_centroid = ransac(points, iterations, distance_threshold);

    // Your code to handle the results...

    return 0;
}
