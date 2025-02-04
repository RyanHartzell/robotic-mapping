#include <iostream>
#include <vector>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"

int main() {
// Set parameters
struct Options {
    int num_features = 1000;
    int num_octave_layers = 3;
    double contrast_threshold = 0.04;
    double edge_threshold = 10;
    double sigma = 1.6;
};
Options options;

// Load images
int num_images = 2;
const std::string image_dir = "../data/";
std::vector<std::string> image_names;
std::vector<cv::Mat> images;

for (int i = 0; i < num_images; ++i) {
    std::string image_name = "img" + std::to_string(i) + ".png";
    image_names.emplace_back(image_name);
    
    std::string image_path = image_dir + image_names[i];
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Error loading image: " << image_path << std::endl;
        return -1;
    }
    images.emplace_back(image);

    cv::imshow(image_name, image);
    cv::waitKey(0);
}

// Create SIFT detector and extractor
auto detector = cv::SIFT::create(options.num_features, options.num_octave_layers,
                                    options.contrast_threshold, options.edge_threshold, options.sigma);
auto extractor = cv::SIFT::create();

std::vector<std::vector<cv::KeyPoint>> keypoints(num_images);
std::vector<cv::Mat> descriptors(num_images);

for (int i = 0; i < num_images; ++i) {
    detector->detect(images[i], keypoints[i]);
    extractor->compute(images[i], keypoints[i], descriptors[i]);

    cv::Mat image_keypoints;
    cv::drawKeypoints(images[i], keypoints[i], image_keypoints);
    cv::imshow("Image with SIFT Keypoints", image_keypoints);
    cv::waitKey(0);
}

// Match descriptors using BFMatcher
cv::BFMatcher matcher(cv::NORM_L2);
std::vector<cv::DMatch> matches;
matcher.match(descriptors[0], descriptors[1], matches);

// Sort matches by distance
std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b) {
    return a.distance < b.distance;
});

// Select top matches
const int num_best_matches = 50;
matches.resize(std::min(num_best_matches, (int)matches.size()));

// Draw matches
cv::Mat match_image;
cv::drawMatches(images[0], keypoints[0], images[1], keypoints[1], matches, match_image);

// Display result
cv::imshow("SIFT Feature Matches", match_image);
cv::waitKey(0);

return 0;
}
