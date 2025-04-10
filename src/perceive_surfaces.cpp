#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <filesystem>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include "ament_index_cpp/get_package_share_directory.hpp"

void process_pcl_data(const std::string& input_pcd, const std::string& output_pcd)
{
  // define the input pointcloud as a pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // read .pcd file
  pcl::PCDReader cloud_reader;
  cloud_reader.read(input_pcd, *cloud);

  // define the output pointcloud as a pointer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // voxel filter (downsampling)
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.0001, 0.0001, 0.0001); // play with (the higher, the larger the distance between the points)
  voxel_filter.filter(*cloud_filtered);

  // plane segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::SACMODEL_PLANE); // search for planes
  seg.setModelType(pcl::SAC_RANSAC); // use RANSAC for plane segmentation
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers_plane, *coefficients_plane);

      // extract plane from image
  pcl::ExtractIndices<pcl::PointXYZ> neg_plane_extracted;
  neg_plane_extracted.setInputCloud(cloud_filtered);
  neg_plane_extracted.setIndices(inliers_plane);
  // neg_plane_extracted.setNegative(true); // removes the plane from the original point cloud
  neg_plane_extracted.setNegative(false); // leaves only the plane and removes everything else
  neg_plane_extracted.filter(*cloud_filtered);

  // Example for estimating bounding box size
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt);
  double length = max_pt.x - min_pt.x;
  double width  = max_pt.y - min_pt.y;
  double height = max_pt.z - min_pt.z;

  std::cout << "Plane coefficients: "
          << coefficients_plane->values[0] << " "
          << coefficients_plane->values[1] << " "
          << coefficients_plane->values[2] << " "
          << coefficients_plane->values[3] << std::endl;

  std::cout << "Bounding Box Dimensions:" << std::endl;
  std::cout << "Length (X): " << length << " meters" << std::endl;
  std::cout << "Width  (Y): " << width  << " meters" << std::endl;
  std::cout << "Height (Z): " << height << " meters" << std::endl;

  // save the filtered .pcd file
  pcl::PCDWriter cloud_writer;
  cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud_filtered, false);
}

int main(int argc, char ** argv)
{
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("pcl_detection");


    // std::string path_input="/home/oskars/workspace/pcl_ws/src/pcl_detection/src/inputs/";
    std::string path_output="/home/oskars/workspace/pcl_ws/src/pcl_detection/data/outputs/";
    std::string path_input = package_share_dir + "/data/inputs/";
    // std::string path_output = package_share_dir + "/data/outputs/";
    std::string input_pcd = path_input + std::string("test.pcd");
    std::string output_pcd = path_output + std::string("filtered_realsense.pcd");

    process_pcl_data(input_pcd, output_pcd);
    return 0;
}
