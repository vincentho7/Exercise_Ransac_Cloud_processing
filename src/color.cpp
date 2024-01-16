#include "color.hh"

std::vector<Eigen::Vector3f> distinctColors = {
        Eigen::Vector3f(1.0f, 0.0f, 0.0f), // Red
        Eigen::Vector3f(0.0f, 1.0f, 0.0f), // Green
        Eigen::Vector3f(0.0f, 0.0f, 1.0f), // Blue
        Eigen::Vector3f(1.0f, 1.0f, 0.0f), // Yellow
        Eigen::Vector3f(1.0f, 0.0f, 1.0f), // Magenta
        Eigen::Vector3f(0.0f, 1.0f, 1.0f), // Cyan
        // Add more colors as needed
};

Eigen::Vector3f generate_random_color() {
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    return Eigen::Vector3f(dist(rng), dist(rng), dist(rng));
}

Eigen::Vector3f generate_color(int colorIndex) {
    if (colorIndex < static_cast<int>(distinctColors.size())) {
        // Use the colorIndex to pick a color and increment it
        Eigen::Vector3f selectedColor = distinctColors[colorIndex];
        return selectedColor;
    } else {
        Eigen::Vector3f newColor;
        newColor = generate_random_color();
        
        distinctColors.push_back(newColor); // Add the new unique color to the list
        return newColor;
    }
}