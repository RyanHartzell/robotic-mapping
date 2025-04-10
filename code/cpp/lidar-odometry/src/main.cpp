#include <iostream>
#include <vector>
#include <string>
#include <memory>

// OpenCV:
#include <opencv2/core.hpp>

// Open3D:
#include "open3d/Open3D.h"

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ------------------------------------------
// Function to perform ICP alignment between two point clouds
Eigen::Matrix4d PerformICPAlignment(
    const std::shared_ptr<open3d::geometry::PointCloud>& source,
    const std::shared_ptr<open3d::geometry::PointCloud>& target,
    double max_distance = 0.1,
    int max_iterations = 30) {
    
    // Estimate normals (required for point-to-plane ICP)
    source->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
    target->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
    
    // ICP parameters
    double threshold = max_distance;
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    
    // Perform ICP registration on full-resolution point clouds
    auto result = open3d::pipelines::registration::RegistrationICP(
        *source, *target, threshold, transformation,
        open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        open3d::pipelines::registration::ICPConvergenceCriteria(
            max_iterations, 1e-6, 1e-6));
    
    return result.transformation_;
}


// ------------------------------------------
int main() {

// pointcloud data
std::string base_name = "cloud_";
const std::string file_dir = "../data/"; 
const int num_files = 5; // number of point clouds in the dataset

// file names
std::vector<std::string> file_names;
for (int i = 0; i < num_files; ++i) {   
    std::string name = base_name + std::to_string(i) + ".ply";
    file_names.emplace_back(name);
}

// Create a vector to store all point clouds
std::vector<std::shared_ptr<open3d::geometry::PointCloud>> point_clouds;

// Load each point cloud
for (const auto& file_name : file_names) {
    std::string full_path = file_dir + file_name;
    auto pcd = open3d::io::CreatePointCloudFromFile(full_path);
    
    if (pcd) {
        std::cout << "Successfully loaded: " << full_path << std::endl;
        point_clouds.push_back(pcd);
    } else {
        std::cerr << "Failed to load: " << full_path << std::endl;
    }
}

if (point_clouds.empty()) {
    std::cerr << "No point clouds loaded. Exiting..." << std::endl;
    return -1;
}

// Visualize each point cloud individually
for (size_t i = 0; i < point_clouds.size(); ++i) {
    open3d::visualization::DrawGeometries({point_clouds[i]}, 
        "Point Cloud: " + file_names[i], 1600, 900);
}


 // Align consecutive point clouds using ICP
 std::vector<Eigen::Matrix4d> transformations;
 transformations.push_back(Eigen::Matrix4d::Identity()); // Identity for first cloud

for (size_t i = 1; i < point_clouds.size(); ++i) {
    std::cout << "\nAligning cloud " << i-1 << " to cloud " << i << std::endl;
    
    // Perform ICP alignment on point clouds
    Eigen::Matrix4d transformation = PerformICPAlignment(
        point_clouds[i-1], point_clouds[i]);
    
    // Print the relative transformation
    std::cout << "Estimated relative transformation:\n" 
            << transformation << std::endl;
    
    // Extract and print translation and rotation components
    Eigen::Vector3d translation = transformation.block<3,1>(0,3);
    Eigen::Matrix3d rotation = transformation.block<3,3>(0,0);
    
    std::cout << "Translation: " << translation.transpose() << std::endl;
    std::cout << "Rotation matrix:\n" << rotation << std::endl;
    
    // Convert rotation matrix to axis-angle for easier interpretation
    Eigen::AngleAxisd angle_axis(rotation);
    std::cout << "Rotation angle: " << angle_axis.angle() * 180.0 / M_PI 
            << " degrees around axis: " << angle_axis.axis().transpose() << std::endl;
    
    transformations.push_back(transformation);
    
    // Visualize the aligned pair
    auto source_transformed = std::make_shared<open3d::geometry::PointCloud>(*point_clouds[i-1]);
    source_transformed->Transform(transformation);
    
    // Color the point clouds for better visualization
    point_clouds[i]->PaintUniformColor(Eigen::Vector3d(1, 0, 0)); // Red (target)
    source_transformed->PaintUniformColor(Eigen::Vector3d(0, 1, 0)); // Green (aligned source)
    
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.push_back(point_clouds[i]);
    geometries.push_back(source_transformed);
    
    open3d::visualization::DrawGeometries(geometries, 
        "Aligned: Cloud " + std::to_string(i-1) + " (green) to Cloud " + std::to_string(i) + " (red)", 
        1600, 900);
}


return 0;
}
