#include "ransac.hh"
#include "color.hh"
namespace RANSAC {
// Function to estimate a plane from three points
    int iterations = 2000; // Number of iterations
    float dist_threshold = 0.3f; // Distance threshold for inliers
    float align_threshold = 0.9f;
    float pointsleft = 0.25f;
    
    void estimate_plane(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, 
                        Eigen::Vector3f &centroid, Eigen::Vector3f &normal) {
        // Compute the centroid and normal of the plane
        centroid = (p1 + p2 + p3) / 3;
        Eigen::Vector3f u = p2 - p1;
        Eigen::Vector3f v = p3 - p1;
        normal = u.cross(v).normalized();
        // if(normal.norm() == 0) return false;
        // return true;
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

    void simple_ransac(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &colors) {
        // Initialize a pseudo-random generator with a random seed
        std::random_device rd;
        std::mt19937 rng(rd());
        std::uniform_int_distribution<int> dist(0, points.size() - 1);
        // inlier = point in our model  
        int best_inlier_count = 0;
        Eigen::Vector3f best_point, best_normal;

        for(int i = 0; i < iterations; ++i) {
            int idx1 = dist(rng), idx2 = dist(rng), idx3 = dist(rng);
            Eigen::Vector3f centroid, normal;
            estimate_plane(points[idx1], points[idx2], points[idx3], centroid, normal);

            // Count inliers
            int inlier_count = 0;
            for(const auto &point : points) {
                if(point_to_plane_distance(point, centroid, normal) < dist_threshold) {
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
        // draw color
        auto color = generate_color(0);
        for(size_t i = 0; i < points.size(); i++) {
            Eigen::Vector3f point = points[i];
            if(point_to_plane_distance(point, best_point, best_normal) < dist_threshold) {
                colors[i] = color;
            }
        }
    }

    float calculate_alignement(const Eigen::Vector3f &normal1, const Eigen::Vector3f &normal2) {
        return std::abs(normal1.dot(normal2));
    }
        std::vector<size_t> ransac(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals, std::vector<size_t> remaining_idx, int colorIndex) {
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
            for(size_t i = 0; i < remaining_idx.size(); i++) {
                Eigen::Vector3f remaining_point = points[remaining_idx[i]];
                if(point_to_plane_distance(remaining_point, centroid, normal) < dist_threshold) {
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
        auto color = generate_color(colorIndex);
        std::vector<size_t> new_remaining_idx;
        for(size_t i = 0; i < remaining_idx.size(); i++) {
            Eigen::Vector3f point = points[remaining_idx[i]];
            if(point_to_plane_distance(point, best_p, best_n) < dist_threshold) {
                colors[remaining_idx[i]] = color;
            }
            else {
                new_remaining_idx.push_back(remaining_idx[i]);
            }
        }
        return new_remaining_idx;
    }

    void ransac_multiple_planes(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals){
        // use of index to select the remaining points
        std::vector<size_t> remaining_idx;
        int color_index = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            remaining_idx.push_back(i);
        }
        while (static_cast<float>(remaining_idx.size()) / static_cast<float>(points.size()) > pointsleft)
        {
            remaining_idx = ransac(points, colors, normals, remaining_idx, color_index);
            color_index++;
        }
    }
    

    std::vector<size_t> ransac_with_normals(const std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals, std::vector<size_t> remaining_idx, int colorIndex) {
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
            for(size_t i = 0; i < remaining_idx.size(); i++) {
                Eigen::Vector3f remaining_point = points[remaining_idx[i]];
                Eigen::Vector3f remaining_normal = normals[remaining_idx[i]];
                // filtering by distance threshold and normal
                if(point_to_plane_distance(remaining_point, centroid, normal) < dist_threshold and calculate_alignement(remaining_normal, normal) >= align_threshold) {
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
        auto color = generate_color(colorIndex);
        std::vector<size_t> new_remaining_idx;
        for(size_t i = 0; i < remaining_idx.size(); i++) {
            Eigen::Vector3f remaining_point = points[remaining_idx[i]];
            Eigen::Vector3f remaining_normal = normals[remaining_idx[i]];
            if(point_to_plane_distance(remaining_point, best_p, best_n) < dist_threshold and calculate_alignement(remaining_normal, best_n) >= align_threshold) {
                colors[remaining_idx[i]] = color;
            }
            else {
                new_remaining_idx.push_back(remaining_idx[i]);
            }
        }
        return new_remaining_idx;
    }

    void ransac_n_mult_planes(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f> &colors, std::vector<Eigen::Vector3f>& normals){
        // use of index to select the remaining points
        std::vector<size_t> remaining_idx;
        int color_index = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            remaining_idx.push_back(i);
        }
        while (static_cast<float>(remaining_idx.size()) / static_cast<float>(points.size()) > pointsleft)
        {
            remaining_idx = ransac_with_normals(points, colors, normals, remaining_idx, color_index);
            color_index++;
        }
    }
}