#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <random>
#include <obj.h>

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
void ransac(const std::vector<Eigen::Vector3f> &points, int iterations, float distance_threshold, std::vector<Eigen::Vector3f> &colors) {
    std::random_device rd;
    // Initialize a Mersenne Twister pseudo-random generator with a random seed from std::random_device
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist(0, points.size() - 1);

    int best_inlier_count = 0;
    Eigen::Vector3f best_p;
    Eigen::Vector3f best_n;

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
            best_p = centroid;
            best_n = normal;
        }
    }

    for(size_t i = 0; i < points.size(); i++) {
        Eigen::Vector3f point = points[i];
        if(point_to_plane_distance(point, best_p, best_n) < distance_threshold) {
            colors[i] = Eigen::Vector3f(1, 0, 0);
        }
    }
}
struct Plane {
    Eigen::Vector3f centroid;
    Eigen::Vector3f normal;
    std::vector<Eigen::Vector3f> inliers;
    Eigen::Vector3f color; // Color for the plane

};

Eigen::Vector3f generate_random_color(std::mt19937 &rng) {
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return Eigen::Vector3f(dist(rng), dist(rng), dist(rng));
}

// Modified RANSAC algorithm to find multiple planes
std::vector<Plane> ransac_multiple_planes(std::vector<Eigen::Vector3f> &points, int iterations, float distance_threshold, int max_planes, std::vector<Eigen::Vector3f> &colors) {
    std::random_device rd;
    // Initialize a Mersenne Twister pseudo-random generator with a random seed from std::random_device
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist(0, points.size() - 1);

    std::vector<Plane> planes;
    std::vector<Eigen::Vector3f> remaining_points = points;

    for(int plane_count = 0; plane_count < max_planes; ++plane_count) {
        int best_inlier_count = 0;
        Plane best_plane;

        for(int i = 0; i < iterations; ++i) {
            int idx1 = dist(rng), idx2 = dist(rng), idx3 = dist(rng);
            Eigen::Vector3f centroid, normal;
            if(!estimate_plane(points[idx1], points[idx2], points[idx3], centroid, normal)) continue;

            std::vector<Eigen::Vector3f> inliers;
            for(const auto &point : remaining_points) {
                if(point_to_plane_distance(point, centroid, normal) < distance_threshold) {
                    inliers.push_back(point);
                }
            }

            if(inliers.size() > best_inlier_count) {
                best_inlier_count = inliers.size();
                best_plane.centroid = centroid;
                best_plane.normal = normal;
                best_plane.inliers = inliers;
            }
        }

        if(best_inlier_count == 0) {
            break; // No more planes can be found
        }
        best_plane.color = generate_random_color(rng);

        // Store the best plane
        planes.push_back(best_plane);

        // Remove inliers from remaining points
        std::vector<Eigen::Vector3f> new_remaining_points;
        for(const auto &point : remaining_points) {
            if(std::find(best_plane.inliers.begin(), best_plane.inliers.end(), point) != best_plane.inliers.end()) {
                // Assign color to inlier points
                size_t index = &point - &remaining_points[0];
                colors[index] = best_plane.color;
            } else {
                new_remaining_points.push_back(point);
            }
        }
        remaining_points = new_remaining_points;
    }

    return planes;
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
    // std::vector<Eigen::Vector3f> colors;
    
    if(not tnp::load_obj(filename, points, normals, colors)) {
        std::cout << "Failed to open input file '" << filename << "'" << std::endl;
        return 1;
    }

    // RANSAC parameters
    int iterations = 100; // Number of iterations
    float distance_threshold = 0.1f; // Distance threshold for inliers
    int max_planes = 5;
    // Run RANSAC
    // ransac(points, iterations, distance_threshold, colors);
    std::vector<Eigen::Vector3f> colors(points.size(), Eigen::Vector3f(0, 0, 0)); // Initialize all colors to black
    std::vector<Plane> detected_planes = ransac_multiple_planes(points, iterations, distance_threshold, max_planes, colors);

    // Your code to handle the results...
    tnp::save_obj("result.obj", points, normals, colors);

    return 0;
}
