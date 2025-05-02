#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <thread>
#include <chrono>
#include <sstream> // Added for std::stringstream used in CSV parsing
#include <stdexcept> // Added for std::stod exception handling

// OpenCV Headers
#include "opencv2/opencv.hpp"
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/utils/filesystem.hpp>

// Open3D Headers
#include <open3d/Open3D.h>
#include <Eigen/Core>

// Helper function to convert cv::Mat (4x4 CV_64F) to Eigen::Matrix4d
Eigen::Matrix4d cvMatToEigen(const cv::Mat& mat) {
    Eigen::Matrix4d eigenMat = Eigen::Matrix4d::Identity();
    if (mat.cols == 4 && mat.rows == 4 && mat.type() == CV_64F) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                eigenMat(i, j) = mat.at<double>(i, j);
            }
        }
    } else if (!mat.empty()) {
        std::cerr << "Warning: cvMatToEigen input matrix ignored (not 4x4 CV_64F)." << std::endl;
    }
    return eigenMat;
}

int main() {
// --- File paths ---
const std::string image_folder = "../data/Synthetic_base/images/";
const std::string image_pattern = image_folder + "*.png";
const std::string calib_file = "../data/Synthetic_base/camera_calibration.json";
const std::string gt_timestamp_file = "../data/Synthetic_base/timings.csv";
const std::string estimated_traj_file = "../data/Synthetic_base/estimated_trajectory.txt";
// const std::string image_folder = "../data/Synthetic_competition/images/";
// const std::string image_pattern = image_folder + "*.png";
// const std::string calib_file = "../data/Synthetic_competition/camera_calibration.json";
// const std::string gt_timestamp_file = "../data/Synthetic_competition/timings.csv";
// const std::string estimated_traj_file = "../data/Synthetic_competition/estimated_trajectory.txt";

// --- Load ground truth timestamps ---
std::vector<double> timestamps;
std::ifstream gt_file(gt_timestamp_file);
if (!gt_file.is_open()) {
    std::cerr << "ERROR: Could not open timestamp file: " << gt_timestamp_file << std::endl;
    return -1;
}
std::string line;

// Skip the header line
if (!std::getline(gt_file, line)) {
     std::cerr << "ERROR: Could not read header line from " << gt_timestamp_file << std::endl;
     gt_file.close();
     return -1;
}

while (std::getline(gt_file, line)) {
    std::stringstream ss(line);
    std::string segment;
    long long timestamp_ns = 0;

    // Get the first segment (timestamp) before the comma
    if (std::getline(ss, segment, ',')) {
        try {
            // Convert string segment to long long for nanoseconds
            timestamp_ns = std::stoll(segment);
            // Convert nanoseconds to seconds (double) and add to vector
            timestamps.push_back(static_cast<double>(timestamp_ns) / 1.0e9);
        } catch (const std::invalid_argument& ia) {
            std::cerr << "Warning: Invalid timestamp format on line: " << line << " (" << ia.what() << "). Skipping." << std::endl;
        } catch (const std::out_of_range& oor) {
            std::cerr << "Warning: Timestamp out of range on line: " << line << " (" << oor.what() << "). Skipping." << std::endl;
        }
    } else if (!line.empty()) { // Handle lines that might not have a comma but aren't empty
         std::cerr << "Warning: Malformed line (no comma found or empty timestamp): " << line << ". Skipping." << std::endl;
    }
    // We only care about the first column (timestamp), so we don't need to read the rest of the line.
}
gt_file.close();

// --- Load camera intrinsics from JSON file ---
double fx, fy, cx_d, cy_d;
int width = 1920, height = 1080;
try {
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "ERROR: Could not open calibration file: " << calib_file << std::endl;
        return -1;
    }
    fs["fx"] >> fx;
    fs["fy"] >> fy;
    fs["cx"] >> cx_d;
    fs["cy"] >> cy_d;
    // Optionally read width and height if present in JSON
    if (!fs["width"].empty()) fs["width"] >> width;
    if (!fs["height"].empty()) fs["height"] >> height;
    fs.release();
} catch (const cv::Exception& e) {
    std::cerr << "Error processing calibration file: " << e.what() << std::endl;
    return -1;
}

// --- Load image files ---
std::vector<std::string> image_files;
cv::glob(image_pattern, image_files, false);
if (image_files.empty()) {
    std::cerr << "ERROR: No image files found matching pattern: " << image_pattern << std::endl;
    return -1;
}
std::sort(image_files.begin(), image_files.end());

if (timestamps.size() != image_files.size()) {
    std::cerr << "ERROR: Timestamp count (" << timestamps.size()
              << ") doesn't match image count (" << image_files.size() << ")" << std::endl;
    // Add a small debug print to see the last few timestamps read vs expected
    if (!timestamps.empty()) {
        std::cerr << "Last few timestamps read: ";
        size_t start_idx = (timestamps.size() > 5) ? timestamps.size() - 5 : 0;
        for (size_t i = start_idx; i < timestamps.size(); ++i) {
            std::cerr << timestamps[i] << " ";
        }
        std::cerr << std::endl;
    }
     if (!image_files.empty()) {
        std::cerr << "Last few image files found: ";
        size_t start_idx = (image_files.size() > 5) ? image_files.size() - 5 : 0;
        for (size_t i = start_idx; i < image_files.size(); ++i) {
            std::cerr << image_files[i] << " ";
        }
        std::cerr << std::endl;
    }

    return -1;
}

// --- Initialize trajectory file ---
FILE* estimated_traj = fopen(estimated_traj_file.c_str(), "wt");
if (!estimated_traj) {
    std::cerr << "ERROR: Could not open trajectory file: " << estimated_traj_file << std::endl;
    return -1;
}
std::cout << "Saving the estimate trajectory to " << estimated_traj_file << std::endl;

// --- VO Parameters ---
bool use_5pt = true;
int min_inlier_num = 50;
double min_inlier_ratio = 0.1;
cv::Mat camera_pose_cv = cv::Mat::eye(4, 4, CV_64F);

// --- Open3D Visualization Setup ---
open3d::visualization::Visualizer visualizer;
visualizer.CreateVisualizerWindow("Open3D Visual Odometry", 1024, 768);
visualizer.GetRenderOption().background_color_ = Eigen::Vector3d(1.0, 1.0, 1.0);
visualizer.GetRenderOption().point_size_ = 2.0;
visualizer.GetRenderOption().line_width_ = 5.0;

auto coord_axes = open3d::geometry::TriangleMesh::CreateCoordinateFrame(1.0);
visualizer.AddGeometry(coord_axes);

auto trajectory_lineset = std::make_shared<open3d::geometry::LineSet>();
std::vector<Eigen::Vector3d> trajectory_points_eigen;
trajectory_points_eigen.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
trajectory_lineset->points_ = trajectory_points_eigen;
visualizer.AddGeometry(trajectory_lineset);

// --- Initial camera visualization ---
Eigen::Matrix3d intrinsic_matrix;
intrinsic_matrix << fx, 0, cx_d, 0, fy, cy_d, 0, 0, 1;
auto camera_frustum = open3d::geometry::LineSet::CreateCameraVisualization(
    width, height, intrinsic_matrix, cvMatToEigen(camera_pose_cv).inverse(), 0.5);
camera_frustum->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0));
visualizer.AddGeometry(camera_frustum);

// --- Write initial pose ---
Eigen::Matrix4d initial_pose = Eigen::Matrix4d::Identity();
Eigen::Quaterniond q_initial(initial_pose.block<3,3>(0,0));
q_initial.normalize();
fprintf(estimated_traj, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
        timestamps[0],
        initial_pose(0,3), initial_pose(1,3), initial_pose(2,3),
        q_initial.x(), q_initial.y(), q_initial.z(), q_initial.w());

// --- Main processing loop ---
cv::Mat gray_prev = cv::imread(image_files[0], cv::IMREAD_GRAYSCALE);
for (size_t frame_idx = 1; frame_idx < image_files.size(); ++frame_idx) {
    cv::Mat img = cv::imread(image_files[frame_idx]);
    if (img.empty()) continue;

    cv::Mat gray;
    if (img.channels() > 1) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img.clone();
    }

    // --- Feature tracking ---
    std::vector<cv::Point2f> pts_prev, pts;
    cv::goodFeaturesToTrack(gray_prev, pts_prev, 2000, 0.01, 10);
    if (pts_prev.empty()) {
        gray_prev = gray.clone();
        continue;
    }

    std::vector<uchar> status;
    cv::Mat err;
    cv::calcOpticalFlowPyrLK(gray_prev, gray, pts_prev, pts, status, err);

    std::vector<cv::Point2f> tracked_pts_prev, tracked_pts;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            tracked_pts_prev.push_back(pts_prev[i]);
            tracked_pts.push_back(pts[i]);
        }
    }

    // --- Pose estimation ---
    cv::Mat E, R, t;
    std::vector<uchar> ransac_mask;
    int inlier_num = 0;
    if (use_5pt) {
        E = cv::findEssentialMat(tracked_pts_prev, tracked_pts, (fx + fy)/2.0,
                                 cv::Point2d(cx_d, cy_d), cv::RANSAC, 0.999, 1.0, ransac_mask);
    }

    if (!E.empty()) {
        inlier_num = cv::recoverPose(E, tracked_pts_prev, tracked_pts, R, t,
                                     (fx + fy)/2.0, cv::Point2d(cx_d, cy_d), ransac_mask);
    }

    double inlier_ratio = tracked_pts.empty() ? 0 :
                          static_cast<double>(inlier_num)/tracked_pts.size();

    // --- Update camera pose ---
    cv::Vec3b info_color(0, 255, 0); // Green
    if (inlier_num > min_inlier_num && inlier_ratio > min_inlier_ratio && !R.empty() && !t.empty()) {
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        R.convertTo(T(cv::Rect(0, 0, 3, 3)), CV_64F);
        t.convertTo(T(cv::Rect(3, 0, 1, 3)), CV_64F);
        camera_pose_cv = camera_pose_cv * T.inv();
        info_color = cv::Vec3b(0, 0, 255); // Red when updated
    }

    // --- Feature visualization ---
    if (img.channels() < 3) {
        cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    for (size_t i = 0; i < tracked_pts.size(); ++i) {
        cv::Vec3b color = (ransac_mask.empty() || i >= ransac_mask.size() || ransac_mask[i] == 0)
                          ? cv::Vec3b(0, 0, 255)  // Red for outliers
                          : cv::Vec3b(0, 255, 0);  // Green for inliers

        cv::line(img, tracked_pts_prev[i], tracked_pts[i], color);
        cv::circle(img, tracked_pts[i], 2, color, -1);
    }

    // --- Display info text ---
    Eigen::Matrix4d current_pose = cvMatToEigen(camera_pose_cv);
    double x = current_pose(0,3), y = current_pose(1,3), z = current_pose(2,3);
    std::string info = cv::format("Frame: %zu/%zu   Inliers: %d (%.1f%%)   Pos: [%.2f, %.2f, %.2f]",
                                  frame_idx+1, image_files.size(), inlier_num,
                                  inlier_ratio*100, x, y, z);
    cv::putText(img, info, cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.2, info_color, 2);
    cv::imshow("Feature Tracking", img);

    // --- Write estimated pose ---
    Eigen::Quaterniond q(current_pose.block<3,3>(0,0));
    q.normalize();
    fprintf(estimated_traj, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
            timestamps[frame_idx],
            current_pose(0,3), current_pose(1,3), current_pose(2,3),
            q.x(), q.y(), q.z(), q.w());

    // --- Update 3D visualization ---
    trajectory_points_eigen.emplace_back(x, y, z);
    if (trajectory_points_eigen.size() > 1) {
        trajectory_lineset->points_ = trajectory_points_eigen;
        trajectory_lineset->lines_.clear();
        for (size_t i = 0; i < trajectory_points_eigen.size()-1; ++i) {
            trajectory_lineset->lines_.emplace_back(i, i+1);
        }
        trajectory_lineset->colors_.assign(trajectory_lineset->lines_.size(), {1,0,0});
        visualizer.UpdateGeometry(trajectory_lineset);
    }

    auto new_frustum = open3d::geometry::LineSet::CreateCameraVisualization(
        width, height, intrinsic_matrix, current_pose.inverse(), 0.5);
    new_frustum->PaintUniformColor(Eigen::Vector3d(0,0,1));
    visualizer.RemoveGeometry(camera_frustum);
    camera_frustum = new_frustum;
    visualizer.AddGeometry(camera_frustum);

    visualizer.ResetViewPoint(true);
    visualizer.PollEvents();
    visualizer.UpdateRender();

    // --- Handle input ---
    int key = cv::waitKey(1);
    if (key == 27) break;  // ESC to exit
    gray_prev = gray.clone();
}

// --- Cleanup ---
fclose(estimated_traj);
cv::destroyAllWindows();
while (visualizer.PollEvents()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
visualizer.DestroyVisualizerWindow();
return 0;
}