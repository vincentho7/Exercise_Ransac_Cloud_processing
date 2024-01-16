#include "ransac.hh"
#include "color.hh"

// Main RANSAC algorithm to find the best plane
void ransac_part1(const std::vector<Eigen::Vector3f> &points, int iterations, float distance_threshold, std::vector<Eigen::Vector3f> &colors) {
    // Initialize a pseudo-random generator with a random seed
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist(0, points.size() - 1);
    // inlier = point in our model
    int best_inlier_count = 0;
    // best point
    Eigen::Vector3f best_point;
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
            best_point = centroid;
            best_normal = normal;
        }
    }
    auto color = generate_color(1);
    for(size_t i = 0; i < points.size(); i++) {
        Eigen::Vector3f point = points[i];
        if(point_to_plane_distance(point, best_point, best_normal) < distance_threshold) {
            colors[i] = color;
        }
    }
}