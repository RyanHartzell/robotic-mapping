#include <iostream> // For std::cout, std::cerr, std::endl
#include <vector>   // For std::vector
#include <string>   // For std::string, std::to_string
#include <limits>   // For std::numeric_limits
#include <cmath>    // For std::abs
#include <memory>   // For std::shared_ptr
#include <fstream>  // For std::ifstream test

// --- Open3D Headers ---
#include <open3d/Open3D.h>  // Includes most common headers
#include <open3d/io/PointCloudIO.h> 
#include <open3d/visualization/utility/DrawGeometry.h> // For  visualization

// --- Eigen Headers ---
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>


// Helper function to convert Open3D PointCloud to Eigen::MatrixXd (Nx3)
Eigen::MatrixXd PointCloudToEigenMatrix(const open3d::geometry::PointCloud& o3d_cloud) {
    Eigen::MatrixXd eigen_points(o3d_cloud.points_.size(), 3);
    for (size_t i = 0; i < o3d_cloud.points_.size(); ++i) {
        eigen_points.row(i) = o3d_cloud.points_[i];
    }
    return eigen_points;
}

// ----------------------------------------------------------------------------
// Custom ICP Implementation 
// ----------------------------------------------------------------------------
int findNearestNeighborIndex(const Eigen::Vector3d& source_point, const Eigen::MatrixXd& target_points) {
    int best_index = -1;
    double min_dist_sq = std::numeric_limits<double>::max();
    // implement function here
    return best_index;
}

Eigen::Matrix4d Arun(const Eigen::MatrixXd& source_assoc, const Eigen::MatrixXd& target_assoc) {
    assert(source_assoc.rows() == target_assoc.rows());
    assert(source_assoc.cols() == 3 && target_assoc.cols() == 3);
    if (source_assoc.rows() < 3) {
         std::cerr << "Warning: Not enough points for SVD (" << source_assoc.rows() << ")" << std::endl;
         return Eigen::Matrix4d::Identity();
    }
    
    // implement algorithm here

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    // transformation.block<3, 3>(0, 0) = R;
    // transformation.block<3, 1>(0, 3) = t;
    return transformation;
}

Eigen::MatrixXd transformPoints(const Eigen::MatrixXd& points, const Eigen::Matrix4d& transformation) {
    Eigen::MatrixXd transformed_points(points.rows(), 3);
    Eigen::Matrix3d R = transformation.block<3, 3>(0, 0);
    Eigen::Vector3d t = transformation.block<3, 1>(0, 3);
    transformed_points = ((R * points.transpose()).colwise() + t).transpose();
    return transformed_points;
}

Eigen::Matrix4d icp(const Eigen::MatrixXd& source_points, const Eigen::MatrixXd& target_points, int max_iterations, double tolerance = 1e-6) {
    Eigen::Matrix4d total_transformation = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd current_source_points = source_points;
    double prev_error = std::numeric_limits<double>::max();

    if(target_points.rows() == 0 || source_points.rows() == 0) {
        std::cerr << "Error: Cannot run ICP with empty point clouds." << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<int> source_indices;
        std::vector<int> target_indices;
        double current_error = 0.0;
        
        // implement algorithm here

        // Eigen::Matrix4d delta_transformation = Arun(source_assoc, target_assoc);
        // current_source_points = transformPoints(current_source_points, delta_transformation);
        // total_transformation = delta_transformation * total_transformation;
        
        std::cout << "Iteration " << iter << ": MSE = " << current_error << std::endl;
        if (std::abs(prev_error - current_error) < tolerance) {
             std::cout << "Converged after " << iter + 1 << " iterations." << std::endl;
             break;
        }
        prev_error = current_error;
         if (iter == max_iterations - 1) {
            std::cout << "Reached max iterations (" << max_iterations << ")." << std::endl;
        }
    }
    return total_transformation;
}
// ----------------------------------------------------------------------------


int main() {
    const int num_clouds = 5;
    const std::string data_dir = "../data/";
    const int icp_iterations = 30;

    // --- Store filenames and calculated absolute poses ---
    std::vector<std::string> cloud_filenames;
    std::vector<Eigen::Matrix4d> absolute_poses; // Stores T_0_i (transform from cloud i to cloud 0)

    // --- Initialize for cloud 0 ---
    std::string initial_filename = data_dir + "cloud_0.ply";
    cloud_filenames.push_back(initial_filename);
    absolute_poses.push_back(Eigen::Matrix4d::Identity()); // Pose of cloud 0 in frame 0 is Identity

    std::shared_ptr<open3d::geometry::PointCloud> prev_o3d_cloud;

    // Load the very first cloud
    std::cout << "Attempting to load: " << initial_filename << std::endl;
    prev_o3d_cloud = open3d::io::CreatePointCloudFromFile(initial_filename);

    if (!prev_o3d_cloud || prev_o3d_cloud->IsEmpty()) {
        std::cerr << "Error: Could not load initial point cloud or it is empty: " << initial_filename << std::endl;
        std::ifstream test_file(initial_filename);
        if (!test_file.good()) std::cerr << "      File does not exist or cannot be opened." << std::endl;
        else std::cerr << "      File exists, but loading failed or resulted in an empty cloud." << std::endl;
        return 1;
    }
     std::cout << "Loaded " << prev_o3d_cloud->points_.size() << " points from " << initial_filename << std::endl;


    // --- Loop through consecutive pairs to find relative transforms and calculate absolute poses ---
    for (int i = 0; i < num_clouds - 1; ++i) {
        std::cout << "\n-----------------------------------------" << std::endl;
        std::cout << "Aligning cloud_" << i << " to cloud_" << (i + 1) << std::endl;
        std::cout << "-----------------------------------------" << std::endl;

        std::shared_ptr<open3d::geometry::PointCloud> target_o3d_cloud;
        std::string target_filename = data_dir + "cloud_" + std::to_string(i + 1) + ".ply";

        std::cout << "Attempting to load: " << target_filename << std::endl;
        target_o3d_cloud = open3d::io::CreatePointCloudFromFile(target_filename);

        if (!target_o3d_cloud || target_o3d_cloud->IsEmpty()) {
            std::cerr << "Error: Could not load target point cloud or it is empty: " << target_filename << std::endl;
            std::cerr << "Skipping alignment for pair (" << i << ", " << i + 1 << ")" << std::endl;
            break; // Stop processing pairs if loading fails
        }
         std::cout << "Loaded " << target_o3d_cloud->points_.size() << " points from " << target_filename << std::endl;

        // Convert Open3D point clouds to Eigen matrices for our ICP function
        Eigen::MatrixXd source_eigen_points = PointCloudToEigenMatrix(*prev_o3d_cloud);
        Eigen::MatrixXd target_eigen_points = PointCloudToEigenMatrix(*target_o3d_cloud);

        // Run ICP: Calculates T_{i+1}_i (transform from source=i to target=i+1)
        Eigen::Matrix4d relative_transform_T_ip1_i = icp(source_eigen_points, target_eigen_points, icp_iterations);

        std::cout << "\nEstimated Relative Transformation (T_" << i + 1 << "_" << i << "):\n"
                  << relative_transform_T_ip1_i << std::endl;

        // Calculate Absolute Pose: T_0_{i+1} = T_0_i * T_i_{i+1}
        // We need T_i_{i+1} = (T_{i+1}_i).inverse()
        Eigen::Matrix4d T_i_ip1 = relative_transform_T_ip1_i.inverse();

        // Get the previous absolute pose T_0_i
        const Eigen::Matrix4d& pose_T_0_i = absolute_poses.back();

        // Calculate the new absolute pose T_0_{i+1}
        Eigen::Matrix4d pose_T_0_ip1 = pose_T_0_i * T_i_ip1;

        // Store the results for cloud i+1
        cloud_filenames.push_back(target_filename);
        absolute_poses.push_back(pose_T_0_ip1);

        // Update the 'previous' cloud for the next iteration
        prev_o3d_cloud = target_o3d_cloud;
    }

    std::cout << "\nICP alignment process finished." << std::endl;

    // --- Calculate and Print Absolute Poses ---
    std::cout << "\n-----------------------------------------" << std::endl;
    std::cout << "Calculated Absolute Poses (T_0_i):" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    for (size_t i = 0; i < absolute_poses.size(); ++i) {
        std::cout << "Pose of cloud_" << i << " in frame of cloud_0 (T_0_" << i << "):\n"
                  << absolute_poses[i] << std::endl;
    }

    // --- Fuse Point Clouds in Frame 0 ---
    std::cout << "\n-----------------------------------------" << std::endl;
    std::cout << "Fusing point clouds in frame of cloud_0..." << std::endl;
    std::cout << "-----------------------------------------" << std::endl;

    // Create the fused cloud object
    open3d::geometry::PointCloud fused_cloud;

    for (size_t i = 0; i < cloud_filenames.size(); ++i) {
        std::cout << "Loading and transforming: " << cloud_filenames[i] << std::endl;
        std::shared_ptr<open3d::geometry::PointCloud> original_cloud_ptr =
            open3d::io::CreatePointCloudFromFile(cloud_filenames[i]);

        if (!original_cloud_ptr || original_cloud_ptr->IsEmpty()) {
            std::cerr << "Warning: Failed to reload cloud " << cloud_filenames[i] << " for fusion. Skipping." << std::endl;
            continue;
        }

        // Apply the absolute transformation T_0_i
        // The Transform method modifies the point cloud in place
        original_cloud_ptr->Transform(absolute_poses[i]);

        // Add the transformed cloud to the fused cloud
        fused_cloud += *original_cloud_ptr; // Use += operator for concatenation
         std::cout << "  Fused " << original_cloud_ptr->points_.size() << " points. Total points: " << fused_cloud.points_.size() << std::endl;
    }

    if (fused_cloud.IsEmpty()) {
        std::cerr << "Error: Fused cloud is empty. Cannot visualize." << std::endl;
        return 1;
    }

    // --- Visualize the Fused Cloud ---
    std::cout << "\n-----------------------------------------" << std::endl;
    std::cout << "Displaying fused point cloud..." << std::endl;
    std::cout << "Close the visualization window to exit." << std::endl;
    std::cout << "-----------------------------------------" << std::endl;

    auto fused_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>(fused_cloud);
    open3d::visualization::DrawGeometries({fused_cloud_ptr}, "Fused Point Cloud (Frame 0)");

    std::cout << "\nVisualization finished." << std::endl;

    return 0;
}