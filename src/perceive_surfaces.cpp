#include <iostream>
#include <filesystem>
#include <math.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
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

#include <rclcpp/rclcpp.hpp>
#include <cstdio>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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
  seg.setDistanceThreshold(0.005); // 0.01
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers_plane, *coefficients_plane);

      // extract plane from image
  pcl::ExtractIndices<pcl::PointXYZ> neg_plane_extracted;
  neg_plane_extracted.setInputCloud(cloud_filtered);
  neg_plane_extracted.setIndices(inliers_plane);
  // neg_plane_extracted.setNegative(true); // removes the plane from the original point cloud
  neg_plane_extracted.setNegative(false); // leaves only the plane and removes everything else
  neg_plane_extracted.filter(*cloud_filtered);

  // passtrough filter
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(cloud_filtered);

  // pass.setFilterFieldName("z"); // filter the z axis values
  // pass.setFilterLimits(0.5, 1.18); // PLAY -> change the z value and check what is filtered off
  // pass.filter(*cloud_filtered);

  // Align plane to horizontal plane
    // 1. Extract plane normal
  Eigen::Vector3f normal(coefficients_plane->values[0],
                         coefficients_plane->values[1],
                         coefficients_plane->values[2]);
  normal.normalize();

    // 2. Define target normal (Z axis)
  Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);

    // 3. Compute initial angle
  float dot_product = normal.dot(z_axis);
  float angle_rad = std::acos(dot_product);
  float angle_deg = angle_rad * 180.0 / M_PI;

  std::cout << "Initial angle from horizontal (Z-axis): " << angle_deg << " degrees" << std::endl;

  if (angle_deg < 5.0) {
    std::cout << "Plane is already approximately horizontal. No transform applied." << std::endl;
  } else {
    // 4. Compute rotation axis
    Eigen::Vector3f axis = normal.cross(z_axis);

    if (axis.norm() < 1e-6) {
      std::cout << "Normal already aligned with Z-axis (upward or downward)." << std::endl;
    } else {
      axis.normalize();

      // 5. Build and apply rotation
      Eigen::AngleAxisf rotation(angle_rad, axis);
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.rotate(rotation);

      pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, transform);
      std::cout << "Plane rotated to horizontal alignment." << std::endl;

      // 6. Recompute normal after transform (transform the original normal)
      Eigen::Vector3f rotated_normal = rotation * normal;
      float angle_after_rad = std::acos(rotated_normal.dot(z_axis));
      float angle_after_deg = angle_after_rad * 180.0 / M_PI;

      std::cout << "Angle from horizontal after rotation: " << angle_after_deg << " degrees" << std::endl;
    }
  }

  // Example for estimating bounding box size
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt);
  double length = max_pt.x - min_pt.x;
  double width  = max_pt.y - min_pt.y;
  double height = max_pt.z - min_pt.z;

  std::cout << "Bounding Box Dimensions:" << std::endl;
  std::cout << "Length (X): " << length << " meters" << std::endl;
  std::cout << "Width  (Y): " << width  << " meters" << std::endl;
  std::cout << "Height (Z): " << height << " meters" << std::endl;

  std::cout << "max_pt (X): " << max_pt.x << " meters" << std::endl;
  std::cout << "min_pt (x): " << min_pt.x << " meters" << std::endl;
  std::cout << "max_pt (Y): " << max_pt.y << " meters" << std::endl;
  std::cout << "min_pt (Y): " << min_pt.y << " meters" << std::endl;
  std::cout << "max_pt (X): " << max_pt.z << " meters" << std::endl;
  std::cout << "min_pt (Z): " << min_pt.z << " meters" << std::endl;


  
  // save the filtered .pcd file
  pcl::PCDWriter cloud_writer;
  cloud_writer.write<pcl::PointXYZ>(output_pcd, *cloud_filtered, false);
}

void publish_collision_plane(
  rclcpp::Node::SharedPtr node, 
  double width = 0.599983, 
  double length = 1.38467, 
  double height = 0.01, 
  pcl::PointXYZ min_pt, 
  pcl::PointXYZ max_pt) 
{

  double width = 0.599983;
  double length = 1.38467;
  double height = 0.01;  // thin height to represent a plane

  pcl::PointXYZ min_pt, max_pt;

  min_pt.x = -0.692448;
  max_pt.x = 0.69222;

  min_pt.y = -0.90901;
  max_pt.y = -0.309027;

  double x = (min_pt.x + max_pt.x) / 2.0;
  double y = (min_pt.y + max_pt.y) / 2.0;
  double z = 1.0;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "segmented_plane";

  // Define the box primitive
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = length;
  primitive.dimensions[1] = width;
  primitive.dimensions[2] = height;

  // Define the pose of the box
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z + height / 2.0;  // so the base is aligned with the plane
  pose.orientation.w = 1.0;  // no rotation since the plane is already aligned horizontally

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  // node->get_service_names_and_types()

  planning_scene_interface.applyCollisionObject(collision_object);
  RCLCPP_INFO(node->get_logger(), "Published collision object.");
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

    // process_pcl_data(input_pcd, output_pcd);

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = rclcpp::Node::make_shared("collision_publisher");
    // rclcpp::spin(node);

    // async node execution
    executor.add_node(node);

    std::thread executor_thread([&executor]() { executor.spin(); });
    executor_thread.detach();

    RCLCPP_INFO(node->get_logger(), "Run publish_collision_plane");
    publish_collision_plane(node);

    while(rclcpp::ok())
    {

    }

    rclcpp::shutdown();
    return 0;
}
